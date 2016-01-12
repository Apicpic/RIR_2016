#include <digitalWriteFast.h>
#include <SimpleTimer.h>
#include <SoftwareSerial.h>
#include <PololuQik.h> 

SimpleTimer timer1;	// def timer de l'echantillonage

PololuQik2s12v10 qik(3,2,4);

/***********************Constante du robot************************************
 Diametre des roues = 5.08 cm
 Perimetre des roues = PI*5.08
 Nombre de ticks par tour => 2400 - 2420
******************************************************************************/

float dist_Roues_Ticks= 130; // distance entre les deux roues en ticks 1709.63 <- valeur theorique
float freqEch=100;
float tickParTour = 2400;

float coeff_Etalonnage_Position=1.0804; // permet de reajuster la pos therorique avec la reelle 
float degreePerRad = 57.295779513; 

/************************Erreur pour calcul PI********************************/

float err_RD;
float err_RG;
float sommeErr_RD;
float sommeErr_RG;

/***********************Position du robot*************************************
 Position en coordonnees cartesiennes X et Y a t, qui seront stocke dans dX et 
 dY lors du calcul de la position suivante
 dist_R et L est la distance parcouru par chaque roue en cm
 theta_ini est l'angle de depart, theta est l'angle du robot a t et dTheta est 
 l'angle du robot par rapport a la derniere position
 ticksCodeurD et G est le nombre de ticks releve pour chaque roues a t
******************************************************************************/

float X, Y;
float dX=0, dY=0;
float dist_R = 0, dist_L = 0; 
float dist, d_dist, pre_dist;

float theta_ini = 0;
float theta, dTheta, theta_pre, theta_moy, theta_moy_rad, dTheta_rad;

long ticksCodeurD=0;
long ticksCodeurG=0;

/************************Vitesse du Robot*************************************/

float vit_Robot; // vitesse du robot
float vit_X; // vitesse du robot projetion sur X
float vit_Y; // vitesse du robot projetion sur Y


unsigned long tpsINI = 0;
unsigned long tpsMAX = 90000;
unsigned long tpsMINLanceFilet = 75000;
int dep;

#define VOIXA_DROITE 18 // Fil BLANC du codeur
#define VOIXB_DROITE 19 // Fil VERT du codeur

/**************Cablage des vecteurs d'interruptions pour le codeur droit*******/

#define vectInterruptA_droit 5 // Va avec VOIXA_DROIT
#define vectInterruptB_droit 4 // Va avec VOIXB_DROIT


#define VOIXA_GAUCHE 20 // Fil BLANC du codeur
#define VOIXB_GAUCHE 21 // Fil VERT du codeur

/**************Cablage des vecteurs d'interruptions pour le codeur gauche******/

#define vectInterruptA_gauche 3 // Va avec VOIXA_GAUCHE
#define vectInterruptB_gauche 2 // Va avec VOIXB_GAUCHE


void setup()
{
  Serial.begin(9600); // ini com PC/Arduino
  Serial.flush();
  delay(100);
  //Echantillonnage
  timer1.setInterval(freqEch, Repeat_Me);
  // Initialisation PololuQik
  qik.init();

/**Définition des pins en mode entrée et activation de la résistance de pull-up*/
  // CODEUR DROIT
  pinMode(VOIXA_DROITE,INPUT);           
  pinMode(VOIXB_DROITE,INPUT);
  digitalWrite(VOIXA_DROITE,HIGH);       
  digitalWrite(VOIXB_DROITE,HIGH);  
  // CODEUR GAUCHE
  pinMode(VOIXA_GAUCHE,INPUT);
  pinMode(VOIXB_GAUCHE,INPUT);
  digitalWrite(VOIXA_GAUCHE,HIGH);        
  digitalWrite(VOIXB_GAUCHE,HIGH);

/**************Attache le changement d'état de la PIN à une fonction************/
  attachInterrupt(vectInterruptA_droit, GestionInterruptionCodeurPinA_DR, RISING);
  attachInterrupt(vectInterruptB_droit, GestionInterruptionCodeurPinB_DR, RISING);
  attachInterrupt(vectInterruptA_gauche, GestionInterruptionCodeurPinA_GA, RISING);
  attachInterrupt(vectInterruptB_gauche, GestionInterruptionCodeurPinB_GA, RISING);
  
  Serial.println("init fini");		
}

void loop()
{	
  qik.setM1Speed(50);
  timer1.run();
}

void Repeat_Me()
{
  Serial.print(ticksCodeurG);
  Serial.print("\t");
  Serial.println(ticksCodeurD);
  Serial.print("\n");
  dist_right();
  dist_left();
  Serial.print(dist_L);
  Serial.print("\t");
  Serial.println(dist_R);
  Serial.print("\n");
}

/* ROUTINE DE SERVICE POUR LES INTERRUPTIONS DES VOIX A ET B DES CODEURS DROIT ET GAUCHE */
//-----------------------------    DROIT     --------------------------------------------//
// Routine de service d'interruption attachée à la voie A du codeur incrémental DROIT
void GestionInterruptionCodeurPinA_DR()
{
 if (digitalReadFast2(VOIXA_DROITE) == digitalReadFast2(VOIXB_DROITE))
  {
    ticksCodeurD++;
  }
  else 
  {
    ticksCodeurD--;
  }
}
// Routine de service d'interruption attachée à la voie B du codeur incrémental DROIT
void GestionInterruptionCodeurPinB_DR()
{
if (digitalReadFast2(VOIXB_DROITE) == digitalReadFast2(VOIXA_DROITE)) 
  {
    ticksCodeurD--;
  }
  else
  {
    ticksCodeurD++;
  }
}

//-----------------------------      GAUCHE       ---------------------------------------//
// Routine de service d'interruption attachée à la voie A du codeur incrémental GAUCHE
void GestionInterruptionCodeurPinA_GA()
{
  if (digitalReadFast2(VOIXA_GAUCHE) == digitalReadFast2(VOIXB_GAUCHE))
  {
    ticksCodeurG--;
  }
  else 
  {
    ticksCodeurG++;
  }
}
// Routine de service d'interruption attachée à la voie B du codeur incrémental GAUCHE
void GestionInterruptionCodeurPinB_GA()
{
  if (digitalReadFast2(VOIXB_GAUCHE) == digitalReadFast2(VOIXA_GAUCHE)) 
  {
    ticksCodeurG++;
  }
  else
  {
    ticksCodeurG--;
  }
}

/**************Calcul de la distance par rapport aux nombres de ticks**********/

void dist_right()
{
  dist_R = ticksCodeurD*(5.08*PI)/2400;
}

void dist_left()
{
  dist_L = ticksCodeurG*(5.08*PI)/2400;
}

/***********************Calcul de x et y ***************************************************
 Cette fonction recupere les 2 variables dist_R et dist_L et actualise les valeurs de X et Y
 Les variables dist_R et dist_L sont actialise par la fonction dist_right() et dist_left()
********************************************************************************************/

void calcul_xy()
{
  dist = (dist_R + dist_L)/2;
  theta = theta_ini + (dist_R - dist_L);
  d_dist = dist - pre_dist;
  dTheta = theta - theta_pre;
  theta_moy = (theta+theta_pre)/2;
  dTheta_rad = degreePerRad * dTheta;
  theta_moy_rad = degreePerRad * theta_moy;
  dX = d_dist * cos(theta_moy_rad);
  dY = d_dist * sin(theta_moy_rad);

  X += dX;
  Y += dY;

  theta_pre = theta;
  pre_dist = dist;
}

