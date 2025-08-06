MODULE Lab_4
! -------------------------------------------------------------------------------------
! Programme : GPA546Lab4
! Auteurs   : Olivier Millette ET Camilo Serna
! Date      : 4 août 2025
! Révision  : V0.0
!
! Description :
!   Module de contrôle d’une cellule robotisée gérant la prise et le dépôt de blocs via une glissoire,
!   la vérification des positions articulaires, la simulation de trajectoire de soudure avec indicateur lumineux,
!   et la configuration des E/S pour commande de pince, vérin et lampes.
! -------------------------------------------------------------------------------------

  ! Positions pré-définies VRAI ROBOT
  PERS robtarget rPriseGli:=[[-660.73,-1058.86,419.28],[0.208833,0.621189,-0.717118,-0.237182],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position glissoire prise
  PERS robtarget rDepotGli:=[[-851.42,-914.68,568.33],[0.211183,0.614,-0.723282,-0.235094],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position glissoire dépôt
  PERS robtarget rDepot:=[[1.48,-964.60,368.18],[0.0173768,0.643389,-0.765217,0.0138353],[-1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position dépôt bloc
  PERS robtarget rRetrait:=[[-380.03,-976.73,740.33],[0.0571998,0.646818,-0.758565,-0.0541672],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position retrait bloc
  PERS robtarget rCrayon:=[[-460.90,-711.35,486.39],[0.00396694,-0.645432,0.763775,0.00698067],[-2,-1,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position crayon
  PERS robtarget rSoudure_1:=[[-175.18,-1041.29,351.25],[0.00391247,-0.645351,0.763845,0.00688849],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Point soudure 1
  PERS robtarget rSoudure_2:=[[-142.57,-837.02,351.29],[0.00391263,-0.645347,0.763849,0.00688715],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Point soudure 2

  ! ***** Vitesses prédéfinies *****
  PERS speeddata VeryLowSpeed := [25, 500, 5000, 1000];

  ! Position calculée
  VAR robtarget rDepot_new;
  VAR robtarget rSoudure_3;

  ! Constantes
  CONST num Epaisseur   := 1;     ! Épaisseur d'un bloc (en pouces)
  CONST num PouceToMM   := 25.4;  ! Facteur de conversion pouce ? mm
  CONST num Decalage    := -200;  ! Distance d'approche/retrait (mm) (+X pour marge de sécurité)

  ! Variables
  VAR num EpaisMM := Epaisseur * PouceToMM;
  VAR num angleDeg := 0;
  VAR pos pos_rRetrait;
  VAR pos pos_Actuelle;
  VAR num calcDistance;

  ! Vitesses d'approche/retrait (mm/s)
  CONST speeddata LowSpeed:=[250,500,5000,1000];
  ! Vitesse max robot (mm/s)
  CONST speeddata HighSpeed:=[1000,500,5000,1000];

  ! États I/O
  CONST dionum Ouverte   := 0;
  CONST dionum Fermee    := 1;
  CONST dionum Retracte  := 0;
  CONST dionum Extension := 1;

  ! Zone de travail restreinte du robot
  VAR wztemporary LimArtic;          ! Supervision de la limite des joints
  VAR wztemporary ZoneProximite;     ! Sphère de 150 mm autour de rRetrait
  VAR shapedata sphProximite;

 ! ----------------------------------------------------------------------------
 ! ----------------------------------------------------------------------------

 ! Sorties numériques (DO) ALIASES
 VAR signaldo pinceFermer;          ! DO01_EE_PINCE01  - Commande de fermeture/ouverture de la pince
 VAR signaldo ventouse1Succion;     ! DO02_EE_VENT01   - Vide (ventouse 1)
 VAR signaldo ventouse2Succion;     ! DO03_EE_VENT02   - Vide (ventouse 2)
 VAR signaldo lampeBleue;           ! DO04_EE_LampBlu  - Lampe bleue (statut)
 VAR signaldo lampeOrange;          ! DO05_EE_LampOr   - Lampe orange (avertissement)
 VAR signaldo verinExtension;       ! DO09_FV0101      - Commande d'extension/rétraction du vérin glissoire

! Entrées numériques (DI) ALIASES
 VAR signaldi objetSurVentouse1;    ! DI01_EE_VAC01    - Objet détecté sur ventouse 1
 VAR signaldi objetSurVentouse2;    ! DI02_EE_VAC02    - Objet détecté sur ventouse 2
 VAR signaldi blocPresentPoussoir;  ! DI09_ZS0101      - Bloc présent dans la glissoire (poussoir)
 VAR signaldi blocOriente;          ! DI10_ZS0102      - Bloc correctement orienté
 VAR signaldi blocBasNiveau;        ! DI11_LSL0101     - Bas niveau de blocs dans la glissoire
 VAR signaldi blocHautNiveau;       ! DI12_LSH0101     - Haut niveau de blocs dans la glissoire
 VAR signaldi verinRetracte;        ! DI13_ZS0103      - Fin de course : vérin rétracté
 VAR signaldi verinEtendu;          ! DI14_ZS0104      - Fin de course : vérin étendu
 VAR signaldi boutonSoudureDI;      ! DI_Virtuel1_Bouton1 - bouton programmable 1
 
!*************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
VAR intnum soudureInterrupt;   ! Any unused interrupt number
VAR bool soudureDemandee := FALSE;
VAR bool lampeBleueActive := FALSE;

TRAP DemandeSoudure
    soudureDemandee := TRUE;
    ! Suspendre jusqu'à IWatch
    ISleep soudureInterrupt;
    TPWrite "Soudure demandée! Sera exécutée après cette Opération.";
ENDTRAP

PROC SimulerSoudure()
    VAR pos vecXY;                                      ! Vecteur raccord
    VAR num vecNorm;                                    ! Norme vecteur
    VAR num transl_x;                                   ! Translation X
    VAR num transl_y;                                   ! Translation Y
    VAR num angle_triangle := 30 * 3.14159265 / 180;    ! Angle en rad
    VAR num angle_soudure := 30;                        ! Angle soudure
    VAR pos dirXY;                                      ! Direction XY unitaire
    VAR robtarget rSoudure_2_calc;                      ! Point 2 recalculé
    VAR robtarget rSoud_3_temp;                         ! Temp. pour 3ème point

    ! Prendre crayon
    LeCrayon\Prise;
    
    !------------------------------------------------------------
    ! Calcul du vecteur “vecXY” reliant rSoudure_1 à rSoudure_2 :
    ! Ce vecteur indique la direction brute du déplacement de soudure,
    ! c’est la base pour définir un point intermédiaire à 50 mm.
    !------------------------------------------------------------
    vecXY := [
        rSoudure_2.trans.x - rSoudure_1.trans.x,
        rSoudure_2.trans.y - rSoudure_1.trans.y,
        rSoudure_2.trans.z - rSoudure_1.trans.z
    ];
    
    !------------------------------------------------------------
    ! Norme du vecteur vecXY (longueur réelle) :
    ! - Pow(...,2) élève chaque composante au carré
    ! - Sqrt(...) calcule la racine carrée de la somme, soit la distance 3D
    !------------------------------------------------------------
    vecNorm := Sqrt(Pow(vecXY.x,2) + Pow(vecXY.y,2) + Pow(vecXY.z,2));
    
    !------------------------------------------------------------
    ! Vecteur unitaire “dirXY” :
    ! - Chaque composante est divisée par la norme
    ! - Résultat : vecteur de direction de longueur 1
    !------------------------------------------------------------
    dirXY := [vecXY.x / vecNorm, vecXY.y / vecNorm, vecXY.z / vecNorm];
    
    !------------------------------------------------------------
    ! Initialise le calcul de rSoudure_2_calc avec la position de base rSoudure_1
    ! Décale rSoudure_2_calc de 50 mm le long de la direction unitaire :
    ! - On multiplie chaque composante unitaire par 50
    ! - On ajoute ce décalage aux coordonnées de rSoudure_1
    ! - Résultat : point situé exactement à 50 mm sur la trajectoire initiale
    !------------------------------------------------------------
    rSoudure_2_calc := rSoudure_1;
    
    rSoudure_2_calc.trans.x := rSoudure_1.trans.x + 50 * dirXY.x;
    rSoudure_2_calc.trans.y := rSoudure_1.trans.y + 50 * dirXY.y;
    rSoudure_2_calc.trans.z := rSoudure_1.trans.z + 50 * dirXY.z;
    
    ! Calcul du troisième point avec la fonction    
    IF RobOS() THEN
        !--- Exécution sur le vrai robot ---
        ! Définir l’angle de translation dans le plan XY
        transl_x := 50 * Cos(angle_triangle);   ! Projection en X de l’arête du triangle
        transl_y := 50 * Sin(angle_triangle);   ! Projection en Y

        ! Appliquer une rotation de -60° autour de l’axe Z sur rSoudure_2_calc
        rSoud_3_temp := RelTool(rSoudure_2_calc, 0, 0, 0 \Rz := -60);
        ! Puis décaler ce point translaté pour obtenir rSoudure_3
        rSoudure_3 := RelTool(rSoud_3_temp, transl_x, -transl_y, 0);
    ELSE
        !--- Exécution sur le contrôleur virtuel (RobotStudio) ---
        angle_soudure := 10;                    ! Angle d’inclinaison réduit pour le virtuel
        transl_x := 50 * Sin(angle_triangle);   ! Inverse Cos/Sin pour la translation X
        transl_y := 50 * Cos(angle_triangle);   ! Inverse Sin/Cos pour la translation Y

        ! Rotation de +60° autour de Z sur rSoudure_2_calc pour simuler le virtuel
        rSoud_3_temp := RelTool(rSoudure_2_calc, 0, 0, 0 \Rz := 60);
        ! Calcul final du troisième point avec traduction
        rSoudure_3 := RelTool(rSoud_3_temp, transl_x, transl_y, 0);
    ENDIF

    !---------------------------------------------------------------------------
    ! Mise en place et exécution de la trajectoire de soudure
    !---------------------------------------------------------------------------

    ! 1) Approche rapide de la position de départ de la soudure
    MoveJ RelTool(rSoudure_1, 0, 0, Decalage), HighSpeed, z40, tCrayon\wobj:=wobj0;
    ! 2) Orientation précise du crayon au bon angle avant soudure
    MoveJ RelTool(rSoudure_1, 0, 0, 0 \Rx := angle_soudure), LowSpeed, fine, tCrayon\wobj:=wobj0;

    ! 3) Signaler le début de la soudure par l’allumage de la lampe orange
    SetDO lampeOrange, 1;
    WaitTime 0.5;  ! Petite pause pour assurer la visibilité

    ! 4) Exécution du contour triangulaire :
    !    • Ligne du point 1 vers le point 2_calc
    MoveL RelTool(rSoudure_2_calc, 0, 0, 0 \Rx := angle_soudure), VeryLowSpeed, fine, tCrayon\wobj:=wobj0;
    !    • Ligne du point 2_calc vers le point 3
    MoveL RelTool(rSoudure_3,        0, 0, 0 \Rx := angle_soudure), VeryLowSpeed, fine, tCrayon\wobj:=wobj0;
    !    • Retour du point 3 vers le point 1
    MoveL RelTool(rSoudure_1,        0, 0, 0 \Rx := angle_soudure), VeryLowSpeed, fine, tCrayon\wobj:=wobj0;

    ! 5) Éteindre la lampe orange pour signaler la fin de la soudure
    SetDO lampeOrange, 0;
    WaitTime 0.5;  ! Pause avant de reposer le crayon

    ! 6) Reposer le crayon et réinitialiser l’état de la demande de soudure
    LeCrayon\Deposer;              ! Dépose le crayon en position de repos
    soudureDemandee := FALSE;      ! Réinitialiser le flag demande soudure
    IWatch soudureInterrupt;       ! Réactiver l’interruption pour la prochaine soudure
    
ENDPROC

PROC LeCrayon(\switch Prise | switch Deposer)
    MoveJ RelTool(rCrayon,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
    MoveL rCrayon, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 0.5;
    IF Present(Prise) THEN
        Pince\Fermer;
    ELSE
        Pince\Ouvert;
    ENDIF
    MoveL RelTool(rCrayon,0,0,Decalage), LowSpeed, fine, tPince_bloc\wobj:=wobj0;
ENDPROC

PROC SurveillanceProximite()
    pos_rRetrait := rRetrait.trans;
    pos_Actuelle := CPos(\Tool:=tPince_bloc, \WObj:=wobj0);
    calcDistance := Distance(pos_Actuelle, pos_rRetrait);

    IF calcDistance < 150 THEN
        IF NOT lampeBleueActive THEN
            SetDO lampeBleue, 1;
            lampeBleueActive := TRUE;
            TPWrite "Trop proche de rRetrait (<150 mm)";
        ENDIF
    ELSE
        IF lampeBleueActive THEN
            SetDO lampeBleue, 0;
            lampeBleueActive := FALSE;
            TPWrite "Éloigné de rRetrait (>150 mm)";
        ENDIF
    ENDIF
ENDPROC


!*************************************************************************************
! ------------------------------------------------------------------------------
!**************************************************************************************
PROC main()
    
	! Initialisation
    configIO;           ! Mappage I/O + contrôles
	init;               ! Configurations initiales
!    Start SurveillanceProximite;
    CONNECT soudureInterrupt WITH DemandeSoudure;
    ISignalDI boutonSoudureDI, low, soudureInterrupt;

    verifPositionAxes;    ! Vérification de positionnement des axes
    
    ! Calculs
	EpaisMM := Epaisseur * PouceToMM;          ! Conversion en mm
    
    ! Movement des blocs
    WHILE TRUE DO
        Deplacement_blocs;
    ENDWHILE
    
	! Aller en position « home »
	MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;

	Pince\Ouvert;                              ! Ouvrir la pince
ENDPROC
!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC configIO()
    !-----------------------------
    ! Mappe les E/S physiques aux alias
    !-----------------------------
    VAR string sigName;   ! Nom du signal en cours de mappage

    ! ==================== SORTIES (DO) ====================
    sigName:="DO01_EE_PINCE01"; AliasIO sigName,pinceFermer;            ! Pince
    sigName:="DO02_EE_VENT01";  AliasIO sigName,ventouse1Succion;       ! Ventouse 1
    sigName:="DO03_EE_VENT02";  AliasIO sigName,ventouse2Succion;       ! Ventouse 2
    sigName:="DO04_EE_LampBlu"; AliasIO sigName,lampeBleue;             ! Lampe bleue
    sigName:="DO05_EE_LampOr";  AliasIO sigName,lampeOrange;            ! Lampe orange
    sigName:="DO09_FV0101";     AliasIO sigName,verinExtension;         ! Vérin glissoire

    ! ==================== ENTRÉES (DI) ====================
    sigName:="DI01_EE_VAC01";   AliasIO sigName,objetSurVentouse1;      ! Capteur vac1
    sigName:="DI02_EE_VAC02";   AliasIO sigName,objetSurVentouse2;      ! Capteur vac2
    sigName:="DI09_ZS0101";     AliasIO sigName,blocPresentPoussoir;    ! Detect bloc
    sigName:="DI10_ZS0102";     AliasIO sigName,blocOriente;            ! Bloc orienté
    sigName:="DI11_LSL0101";    AliasIO sigName,blocBasNiveau;          ! Niveau bas
    sigName:="DI12_LSH0101";    AliasIO sigName,blocHautNiveau;         ! Niveau haut
    sigName:="DI13_ZS0103";     AliasIO sigName,verinRetracte;          ! Fin course rétracté
    sigName:="DI14_ZS0104";     AliasIO sigName,verinEtendu;            ! Fin course étendu
    sigName:="DI_Virtuel1_Bouton1"; AliasIO sigName,boutonSoudureDI;    ! Bouton soudure

    RETURN;                       ! Tout s'est bien passé ? on sort
ERROR
    TPWrite "AliasIO échoué pour : " + sigName;
    TPWrite "ERRNO = " \Num:=ERRNO;
    Exit;                         ! Arrêt du programme en cas d’échec
ENDPROC
!*************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC init()
	! Volume limitant les articulations du robot
	VAR shapedata joint_space;  
    VAR jointtarget low_pos;
    VAR jointtarget high_pos;
    
    IF RobOS() THEN
        ! *** Robot 2
    	low_pos:=[[ -134, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    	high_pos:=[[ -65, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    ELSE
        ! Code for Virtual Controller
        TPWrite "Utilisation de la cellule virtuelle dans Robostudio";
    	low_pos:=[[ 44, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    	high_pos:=[[ 118, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
    ENDIF
	
    ! Activation de la limitation (obligatoire)
    ! ---------- Limitation de l'espace joint ----------
    WZFree  LimArtic;
    WZLimJointDef \Outside, joint_space, low_pos, high_pos;
    WZLimSup      \Temp,    LimArtic,   joint_space;

    ! ---------- Sphère de proximité ----------------
    WZFree  ZoneProximite;
    WZSphDef \Inside, sphProximite, rRetrait.trans, 150;
    WZDOSet  \Temp,   ZoneProximite \Inside, sphProximite, lampeBleue, 1;

    ! Limitation de vitesse pour la sécurité
	VelSet 25,1000;
    ! Rétracter le vérin
	SetDO verinExtension, Retracte; ! ? alias au lieu de DO09_FV0101
	! Ouvrir la pince
    Pince\Ouvert;
    
ENDPROC
!**************************************************************************************
!**************************************************************************************
PROC verifPositionAxes()
    !---------------------------------------------------------------------------
    ! PROC verifPositionAxes
    !   Vérifie la conformité des axes J1–J6 avec la position de référence.
    !---------------------------------------------------------------------------

    ! Déclaration des variables
    VAR jointtarget posActuelle;                    ! Angles actuels J1–J6
    VAR jointtarget posReference;                   ! Angles cibles calculés
    VAR num delta;                                  ! Écart angulaire pour chaque axe
    VAR bool erreurDetectee := FALSE;               ! True si au moins un écart > seuil
    VAR string msgErreur := "Écart articulaire : "; ! Message agrégé des écarts
    VAR num i;                                      ! Compteur boucle axes
    VAR string tmp;                                 ! Fragment de message pour axe
    VAR num seuil;                                  ! Tolérance selon axe

    ! Lecture de la configuration articulaire actuelle
    posActuelle := CJointT();                       ! Récupère les angles joints

    ! Calcul de la configuration articulaire cible à partir de rRetrait
    posReference := CalcJointT(rRetrait, tPince_bloc\wobj:=wobj0);

    ! Boucle de comparaison pour chaque axe J1 à J6
    FOR i FROM 1 TO 6 DO
        ! Calcul de l’écart absolu entre actuel et référence
        IF i = 1 THEN
            delta := fun_abs_val(posActuelle.robax.rax_1 - posReference.robax.rax_1);
        ELSEIF i = 2 THEN
            delta := fun_abs_val(posActuelle.robax.rax_2 - posReference.robax.rax_2);
        ELSEIF i = 3 THEN
            delta := fun_abs_val(posActuelle.robax.rax_3 - posReference.robax.rax_3);
        ELSEIF i = 4 THEN
            delta := fun_abs_val(posActuelle.robax.rax_4 - posReference.robax.rax_4);
        ELSEIF i = 5 THEN
            delta := fun_abs_val(posActuelle.robax.rax_5 - posReference.robax.rax_5);
        ELSE
            delta := fun_abs_val(posActuelle.robax.rax_6 - posReference.robax.rax_6);
        ENDIF

        ! Choix du seuil de tolérance (1° pour J1–J3, 5° pour J4–J6)
        IF i <= 3 THEN
            seuil := 1;
        ELSE
            seuil := 5;
        ENDIF
        
        ! Si l’écart dépasse le seuil, ajouter à msgErreur et marquer l’erreur
        IF delta > seuil THEN
            tmp := "J" + numToStr(i,0) + " (" + numToStr(delta,1) + "°) ";
            msgErreur := msgErreur + tmp;           ! Concatène "Jx (y°)"
            erreurDetectee := TRUE;                 ! Active le flag d’erreur
        ENDIF
    ENDFOR

    ! Rapport final
    IF erreurDetectee THEN
        TPWrite msgErreur;                      ! Affiche axes et écarts dépassés
        TPWrite "Position initiale invalide. Programme arrêté.";
        ! Exit;                                  ! Optionnel : stoppe l’exécution
    ELSE
        TPWrite "Position initiale validée.";   ! Confirmation si tout est OK
    ENDIF
ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Prise_Glissoire()
    !---------------------------------------------------------------------------
    ! PROC Prise_Glissoire
    !   Gère la prise d’un bloc depuis la glissoire :
    !   1) Vérifie si un bloc est présent  
    !   2) En cas d’absence, affiche un message et attend l’arrivée d’un bloc  
    !   3) Approche, saisit et verrouille le bloc avec le vérin et la pince  
    !---------------------------------------------------------------------------

    ! 1) Vérifier la présence d'un bloc dans la glissoire  
    IF DInput(blocPresentPoussoir) = 0 THEN  
        TPErase;  
        TPWrite "Aucun bloc dans la glissoire";  
        TPWrite "Placer deux blocs et remettre en mode AUTO";  
        ! Si pas de bloc, attendre la détection  
        WaitDI blocPresentPoussoir, 1;      ! Bloque jusqu’à passage à 1  
        WaitTime 1;                         ! Petite pause pour stabiliser le bloc  
    ENDIF

    ! 2) Approche rapide vers la position de prise avec offset de sécurité  
    MoveJ RelTool(rPriseGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;  
    ! 3) Indexer (avancer) le vérin pour caler le bloc contre la pince  
    SetDO verinExtension, Extension;       ! Extension du vérin  
    WaitDI verinEtendu, 1;                 ! Attendre la fin de course  

    ! 4) Descente linéaire pour position précise de la pince autour du bloc  
    MoveL rPriseGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;  
    ! 5) Fermer la pince pour saisir le bloc  
    Pince\Fermer;                          ! Action pince fermée  

    ! 6) Rentrer le vérin pour libérer la pièce et permettre le retrait  
    SetDO verinExtension, Retracte;        ! Rétraction du vérin  
    WaitDI verinEtendu, 0;                 ! Attendre la fin de rétraction  

    ! 7) Retrait du robot vers position d’approche hors zone de collision  
    MoveL RelTool(rPriseGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;  
ENDPROC


PROC Depot_Glissoire()
    !---------------------------------------------------------------------------
    ! PROC Depot_Glissoire
    !   Dépose un bloc dans la glissoire :
    !   1) Approche la position de dépôt  
    !   2) Descend pour ouvrir la pince  
    !   3) Recule pour sortir de la zone de travail  
    !---------------------------------------------------------------------------

    ! 1) Approche rapide vers la position de dépôt avec offset  
    MoveJ RelTool(rDepotGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;  
    ! 2) Descente linéaire pour un positionnement précis devant la glissoire  
    MoveL rDepotGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;  

    ! 3) Ouvrir la pince pour libérer le bloc dans la glissoire  
    Pince\Ouvert;                          ! Action pince ouverte  

    ! 4) Reculer en mode linéaire avec offset pour libérer la zone  
    MoveL RelTool(rDepotGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;  
ENDPROC


!**************************************************************************************
! ----------------------------------------------------------------------------
! Procédure : Depot
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Depot(robtarget rPosDepot)

	! 1) Approche rapide offset Z pour éviter les collisions
	MoveJ RelTool(rPosDepot,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;

	! 2) Descente linéaire à vitesse réduite pour un positionnement précis
	MoveL rPosDepot, LowSpeed, fine, tPince_bloc;

	! 3) Ouvrir la pince pour relâcher le bloc
	Pince\Ouvert;

	! 4) Remontée linéaire vers la position d'approche (offset Z)
	MoveL RelTool(rPosDepot,0,0,Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

PROC Prise_en_Depot(robtarget rPrisDepot)

	! 1) Approche rapide offset Z pour éviter les collisions
	MoveJ RelTool(rPrisDepot,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;

	! 2) Descente linéaire à vitesse réduite pour un positionnement précis
	MoveL rPrisDepot, LowSpeed, fine, tPince_bloc;

	! 3) Ouvrir la pince pour relâcher le bloc
	Pince\Fermer;

	! 4) Remontée linéaire vers la position d'approche (offset Z)
	MoveL RelTool(rPrisDepot,0,0,Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Pince(\switch Ouvert | switch Fermer)
    ! Action sur la pince
    IF Present(Fermer) THEN
        ! Vérifier si la sortie doit être modifiée
        IF DOutput(pinceFermer) = 0 THEN
            Set pinceFermer;              ! Fermer la pince
            WaitTime 1;                   ! Attendre le mouvement physique
        ENDIF
    ELSE
        ! Vérifier si la sortie doit être modifiée
        IF DOutput(pinceFermer) = 1 THEN
            Reset pinceFermer;            ! Ouvrir la pince
            WaitTime 1;                   ! Attendre le mouvement physique
        ENDIF
    ENDIF
ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Deplacement_blocs()
    VAR num bloc;
    VAR num num_blocs := 2;
    VAR num distance_rDepot;
    VAR num distance_rPrise;
    
    FOR bloc FROM 0 TO (num_blocs - 1) DO
        distance_rDepot := -((EpaisMM) * bloc);
        ! Calcul de la nouvelle position rDepot_new par rapport à rDepot et au bloc qui correspond
        rDepot_new := RelTool(rDepot, 0, 0, distance_rDepot \Rz := 0);
        ! Prendre le bloc au Glissoire
        Prise_Glissoire;
    	! Dépôt du bloc dans rDepot_new
    	Depot(rDepot_new);
        IF soudureDemandee = TRUE THEN
            SimulerSoudure;
        ENDIF
    ENDFOR
    
    ! Retour à la position de retrait
    MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 3;
    IF soudureDemandee = TRUE THEN
        SimulerSoudure;
    ENDIF
    
    FOR bloc FROM (num_blocs - 1) TO 0 STEP -1 DO
        distance_rPrise := -((EpaisMM) * bloc);
        ! Calcul de la nouvelle position rDepot_new par rapport à rDepot et au bloc qui correspond
        rDepot_new := RelTool(rDepot, 0, 0, distance_rPrise \Rz := 0);
        ! Pris le Bloc le plus a haut
        Prise_en_Depot(rDepot_new);
        ! Depot le bloc au haut du Glissoire
        Depot_Glissoire;
        IF soudureDemandee = TRUE THEN
            SimulerSoudure;
        ENDIF
    ENDFOR
    
    ! Retour à la position de retrait
    MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
    WaitTime 3;
    IF soudureDemandee = TRUE THEN
        SimulerSoudure;
    ENDIF
    
ENDPROC

!**************************************************************************************
!**************************************************************************************
! FUNC num fun_abs_val(num x)
!     Calcule et renvoie la valeur absolue de l’argument x.
!     - Si x est positif ou nul, renvoie x tel quel.
!     - Si x est négatif, renvoie son opposé (-x).
!     Utilisée pour comparer des écarts ou distances sans tenir compte du signe.
!**************************************************************************************
 FUNC num fun_abs_val(num x)
     IF x >= 0 THEN
         RETURN x;
     ELSE
         RETURN -x;
     ENDIF
 ENDFUNC

!**************************************************************************************
!**************************************************************************************
!**************************************************************************************
ENDMODULE
