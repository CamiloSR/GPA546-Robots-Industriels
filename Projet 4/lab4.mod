MODULE Lab4
! -------------------------------------------------------------------------------------
! Programme : GPA546Lab4
! Auteurs   : Olivier Millette ET Camilo Serna
! Date      : 4 août 2025
! Révision  : V0.0
!
! Description :
! -------------------------------------------------------------------------------------

!- Définition du TCP pour l'outil pince-bloc
  PERS tooldata tPince_bloc:= [ TRUE, [[97.4, 0, 223.1], [0.924, 0, 0.383 ,0]], [5, [23, 0, 75], [1, 0, 0, 0], 0, 0, 0]];

! Positions pré-définies
  PERS robtarget rGlissoire_prise:=[[-133.98,664.69,606.56],[0.00871289,-0.963775,-0.00269915,0.266559],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rGlissoire_depot:=[[-327.61,803.76,752.16],[0.00242824,-0.963786,0.0148964,0.26625],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rDepot:=[[811.89,-441.73,584.30],[0.148667,-0.719482,0.659588,-0.158704],[-1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
  PERS robtarget rRetrait:=[[0,0,0],[1,0,0,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];

  ! Position calculée
  VAR robtarget rDepot2;

  ! Constantes
  CONST num Epaisseur   := 1;     ! Épaisseur d'un bloc (en pouces)
  CONST num PouceToMM   := 25.4;  ! Facteur de conversion pouce ? mm
  CONST num Decalage    := -200;  ! Distance d'approche/retrait (mm) (+X pour marge de sécurité)

  ! Variables
  VAR num EpaisMM:=0;

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
  VAR wztemporary EspaceRestreint;

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


!**************************************************************************************
PROC main()
	! 1) Initialisation
    configIO;   ! ? mappage I/O + contrôles
	init;
    verPositionAxes;

	! Aller en position « home »
	MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;

	! 2) Calculs
	EpaisMM := Epaisseur * PouceToMM;          ! Conversion en mm
	rDepot2 := Offs(rDepot,0,0,-EpaisMM);      ! Position pour le 2? bloc

	! 3) Déplacement du premier bloc
	Prise;                                     ! Prendre le bloc
	Depot(rDepot);                             ! Dépôt du bloc 1

	! 4) Déplacement du second bloc
	Prise;                                     ! Prendre le bloc
	Depot(rDepot2);                            ! Dépôt du bloc 2

	! 5) Retour à la position de retrait
	MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
	
	Pince\Ouvert;                              ! Ouvrir la pince
ENDPROC
!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC configIO()

    VAR string sigName;   ! Nom du signal en cours de mappage

    ! ==================== SORTIES (DO) ====================
    sigName := "DO01_EE_PINCE01";
    AliasIO sigName, pinceFermer;

    sigName := "DO02_EE_VENT01";
    AliasIO sigName, ventouse1Succion;

    sigName := "DO03_EE_VENT02";
    AliasIO sigName, ventouse2Succion;

    sigName := "DO04_EE_LampBlu";
    AliasIO sigName, lampeBleue;

    sigName := "DO05_EE_LampOr";
    AliasIO sigName, lampeOrange;

    sigName := "DO09_FV0101";
    AliasIO sigName, verinExtension;


    ! ==================== ENTRÉES (DI) ====================
    sigName := "DI01_EE_VAC01";
    AliasIO sigName, objetSurVentouse1;

    sigName := "DI02_EE_VAC02";
    AliasIO sigName, objetSurVentouse2;

    sigName := "DI09_ZS0101";
    AliasIO sigName, blocPresentPoussoir;

    sigName := "DI10_ZS0102";
    AliasIO sigName, blocOriente;

    sigName := "DI11_LSL0101";
    AliasIO sigName, blocBasNiveau;

    sigName := "DI12_LSH0101";
    AliasIO sigName, blocHautNiveau;

    sigName := "DI13_ZS0103";
    AliasIO sigName, verinRetracte;

    sigName := "DI14_ZS0104";
    AliasIO sigName, verinEtendu;

    RETURN;                       ! Tout s'est bien passé ? on sort
ERROR
    TPWrite "AliasIO échoué pour : " + sigName;
    TPWrite "ERRNO = " \Num:=ERRNO;
    Stop;                         ! Arrêt du programme pour investigation
ENDPROC
!*************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC init()

	! Volume limitant les articulations du robot
	VAR shapedata joint_space;
	
	! Conserver seulement le jeu adapté à votre robot
	! Ces limites logicielles garantissent la sécurité
	
	! *** Robot 1
	!CONST jointtarget low_pos:=[[ 54, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ 128, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** Robot 2
	!CONST jointtarget low_pos:=[[ -134, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ -65, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** Robot 3
	CONST jointtarget low_pos:=[[ 44, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	CONST jointtarget high_pos:=[[ 118, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	
	! *** Robot 4
	!CONST jointtarget low_pos:=[[ -104, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
	!CONST jointtarget high_pos:=[[ -28, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];

	! Activation de la limitation (obligatoire)
	WZFree EspaceRestreint;
	WZLimJointDef \Outside, joint_space, low_pos, high_pos;
	WZLimSup \Temp, EspaceRestreint, joint_space;

    ! Limitation de vitesse pour la sécurité
	VelSet 25,1000;
    ! Rétracter le vérin
	SetDO verinExtension, Retracte; ! ? alias au lieu de DO09_FV0101
	! Ouvrir la pince
    Pince\Ouvert;

ENDPROC
!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
!**************************************************************************************
PROC verPositionAxes()
    ! Déclaration des variables
    VAR jointtarget posActuelle;          ! Position articulaire actuelle du robot
    VAR jointtarget posReference;         ! Position articulaire de référence (calculée à partir de rRetrait)
    VAR num delta;                        ! Écart angulaire entre la position actuelle et la référence
    VAR bool erreurDetectee := FALSE;     ! Indicateur s'il y a une erreur sur au moins une articulation
    VAR string msgErreur := "Écart articulaire : ";  ! Message à afficher en cas d'erreur
    VAR num i;

    ! Obtenir la position actuelle des articulations du robot
    posActuelle := CJointT();

    ! Calculer la position articulaire cible à partir du robtarget rRetrait
    posReference := CalcJointT(rRetrait, tPince_bloc\wobj:=wobj0);

    ! Comparaison articulation par articulation (de J1 à J6)
    FOR i FROM 1 TO 6 DO
        IF i = 1 THEN
            delta := Abs(posActuelle.robax.rax_1 - posReference.robax.rax_1);
        ELSEIF i = 2 THEN
            delta := Abs(posActuelle.robax.rax_2 - posReference.robax.rax_2);
        ELSEIF i = 3 THEN
            delta := Abs(posActuelle.robax.rax_3 - posReference.robax.rax_3);
        ELSEIF i = 4 THEN
            delta := Abs(posActuelle.robax.rax_4 - posReference.robax.rax_4);
        ELSEIF i = 5 THEN
            delta := Abs(posActuelle.robax.rax_5 - posReference.robax.rax_5);
        ELSEIF i = 6 THEN
            delta := Abs(posActuelle.robax.rax_6 - posReference.robax.rax_6);
        ENDIF

        ! Si l'articulation fait partie de J1-J3 et dépasse 1 degré, erreur
        IF (i <= 3 AND delta > 1) THEN
            msgErreur := msgErreur + "J" + ValToStr(i) + " (" + ValToStr(delta) + "°) ";
            erreurDetectee := TRUE;
        ENDIF

        ! Si l'articulation fait partie de J4-J6 et dépasse 5 degrés, erreur
        IF (i > 3 AND delta > 5) THEN
            msgErreur := msgErreur + "J" + ValToStr(i) + " (" + ValToStr(delta) + "°) ";
            erreurDetectee := TRUE;
        ENDIF
    ENDFOR

    ! Si une ou plusieurs erreurs ont été détectées :
    IF erreurDetectee THEN
        TPWrite msgErreur;                              ! Afficher les articulations fautives avec écarts
        TPWrite "Position initiale invalide. Programme arrêté.";
        Stop;                                           ! Arrêter l'exécution du programme
    ELSE
        TPWrite "Position initiale validée.";           ! Aucun problème détecté
    ENDIF
ENDPROC

!**************************************************************************************
! ----------------------------------------------------------------------------
!**************************************************************************************
PROC Prise()

    ! Vérifier la présence d'un bloc dans la glissoire
    IF DInput(blocPresentPoussoir) = 0 THEN
        TPErase;
        TPWrite "Aucun bloc dans la glissoire";
        TPWrite "Placer deux blocs et remettre en mode AUTO";
        WaitDI blocPresentPoussoir, 1;      ! Attendre qu'un bloc soit détecté
        WaitTime 1;                         ! Stabilisation du bloc
    ENDIF

    ! Saisir le bloc
    MoveJ RelTool(rGlissoire_depot, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
    SetDO verinExtension, Extension;       ! Indexer le bloc pendant le mouvement
    WaitDI verinEtendu, 1;                 ! Attendre l'extension du vérin
    MoveL rGlissoire_depot, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    Pince\Fermer;                          ! Fermer la pince
    SetDO verinExtension, Retracte;        ! Rétracter le vérin
    WaitDI verinEtendu, 0;                 ! Attendre la rétraction complète
    MoveL RelTool(rGlissoire_depot, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;

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
!**************************************************************************************
!**************************************************************************************
ENDMODULE
