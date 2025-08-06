MODULE Lab_4
    ! -------------------------------------------------------------------------------------
    ! Programme : GPA546Lab4
    ! Auteurs   : Olivier Millette ET Camilo Serna
    ! Date      : 6 août 2025
    ! Révision  : V0.0
    !
    ! Description :
    !   Module de contrôle d'une cellule robotisée gérant la prise et le dépôt de blocs via une glissoire,
    !   la vérification des positions articulaires, la simulation de trajectoire de soudure avec indicateur lumineux,
    !   et la configuration des E/S pour commande de pince, vérin et lampes.
    ! -------------------------------------------------------------------------------------
    
    ! =====================================================================================
    !   Déclarations des variables et constantes du Module Virtuelle
    ! =====================================================================================
!    ! ---------- Positions cibles persistantes (PERS) - VIRTUAL ROBOT ----------
!    PERS robtarget rPriseGli:=[[-248.62,671.10,350.13],[0.204826,-0.685379,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];          ! Position glissoire prise
!    PERS robtarget rDepotGli:=[[-432.49,814.95,513.48],[0.204826,-0.68538,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];           ! Position glissoire dépôt
!    PERS robtarget rDepot:=[[201.58,790.42,312.21],[0.00756763,-0.709954,-0.704175,-0.00677525],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];        ! Position dépôt bloc
!    PERS robtarget rRetrait:=[[201.58,790.42,648.19],[0.0075676,-0.709954,-0.704175,-0.0067752],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];        ! Position retrait bloc
!    PERS robtarget rCrayon:=[[-66.69,1008.93,431.12],[0.0114044,-0.709903,-0.704128,-0.0105808],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];       ! Position crayon
!    PERS robtarget rSoudure_1:=[[53.09,671.46,299.54],[0.0075679,-0.709954,-0.704175,-0.00677558],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];      ! Point soudure 1
!    PERS robtarget rSoudure_2:=[[47.57,868.31,299.54],[0.00756788,-0.709954,-0.704175,-0.00677555],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];     ! Point soudure 2

    ! =====================================================================================
    !   DECLARATIONS GLOBALES DU MODULE
    ! =====================================================================================
    ! PERSISTENT ROBOT TARGETS (positions pre-definies) - Pour le VRAI ROBOT
    PERS robtarget rPriseGli   := [[-248.62,671.1,350.13],[0.204826,-0.685379,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position glissoire prise
    PERS robtarget rDepotGli   := [[-432.49,814.95,513.48],[0.204826,-0.68538,-0.673775,0.18528],[1,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position glissoire dépôt
    PERS robtarget rDepot      := [[201.58,790.42,312.21],[0.00756763,-0.709954,-0.704175,-0.00677525],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position dépôt bloc
    PERS robtarget rRetrait    := [[201.58,790.42,648.19],[0.0075676,-0.709954,-0.704175,-0.0067752],[0,0,-2,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position retrait bloc
    PERS robtarget rCrayon     := [[-66.69,1008.93,431.12],[0.0114044,-0.709903,-0.704128,-0.0105808],[1,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Position crayon
    PERS robtarget rSoudure_1  := [[-175.18,-1041.29,351.25],[0.00391247,-0.645351,0.763845,0.00688849],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Point soudure 1
    PERS robtarget rSoudure_2  := [[-142.57,-837.02,351.29],[0.00391263,-0.645347,0.763849,0.00688715],[-2,-1,-1,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]]; ! Point soudure 2

    ! PERSISTENT SPEED DATA (vitesses pre-definies)
    PERS speeddata VeryLowSpeed := [25, 500, 5000, 1000];   ! Vitesse pour la soudure
    PERS speeddata LowSpeed     := [250,500,5000,1000];   ! Vitesse pour approche/retrait precis
    PERS speeddata HighSpeed    := [1000,500,5000,1000];  ! Vitesse max robot

    ! VARIABLES (positions calculees)
    VAR robtarget rDepot_new;
    VAR robtarget rSoudure_3;

    ! CONSTANTES
    CONST num Epaisseur    := 1;      ! Épaisseur d'un bloc (en pouces)
    CONST num PouceToMM    := 25.4;   ! Facteur de conversion pouce -> mm
    CONST num Decalage     := -200;   ! Distance d'approche/retrait (mm) (-Z de l'outil)

    ! VARIABLES GLOBALES
    VAR num EpaisMM := Epaisseur * PouceToMM; ! Valeur de l'epaisseur convertie en mm
    VAR pos pos_rRetrait;
    VAR pos pos_Actuelle;
    VAR num calcDistance;
    VAR string sigName; 
    VAR triggdata trig_LightOn;
    VAR triggdata trig_LightOff;

    ! CONSTANTES POUR ETATS I/O (ameliore la lisibilite)
    CONST dionum Ouverte   := 0;
    CONST dionum Fermee    := 1;
    CONST dionum Retracte  := 0;
    CONST dionum Extension := 1;

    ! ZONE DE TRAVAIL (Work Zones)
    VAR wztemporary LimArtic;         ! Supervision de la limite des joints
    VAR wztemporary ZoneProximite;    ! Sphère de 150 mm autour de rRetrait
    VAR shapedata sphProximite;

    ! ALIASES POUR SORTIES NUMERIQUES (DO)
    VAR signaldo pinceFermer;         ! DO01_EE_PINCE01  - Commande de fermeture/ouverture de la pince
    VAR signaldo ventouse1Succion;    ! DO02_EE_VENT01   - Vide (ventouse 1)
    VAR signaldo ventouse2Succion;    ! DO03_EE_VENT02   - Vide (ventouse 2)
    VAR signaldo lampeBleue;          ! DO04_EE_LampBlu  - Lampe bleue (statut)
    VAR signaldo lampeOrange;         ! DO05_EE_LampOr   - Lampe orange (avertissement)
    VAR signaldo verinExtension;      ! DO09_FV0101      - Commande d'extension/rétraction du vérin glissoire

    ! ALIASES POUR ENTREES NUMERIQUES (DI)
    VAR signaldi objetSurVentouse1;   ! DI01_EE_VAC01    - Objet détecté sur ventouse 1
    VAR signaldi objetSurVentouse2;   ! DI02_EE_VAC02    - Objet détecté sur ventouse 2
    VAR signaldi blocPresentPoussoir; ! DI09_ZS0101      - Bloc présent dans la glissoire (poussoir)
    VAR signaldi blocOriente;         ! DI10_ZS0102      - Bloc correctement orienté
    VAR signaldi blocBasNiveau;       ! DI11_LSL0101     - Bas niveau de blocs dans la glissoire
    VAR signaldi blocHautNiveau;      ! DI12_LSH0101     - Haut niveau de blocs dans la glissoire
    VAR signaldi verinRetracte;        ! DI13_ZS0103      - Fin de course : vérin rétracté
    VAR signaldi verinEtendu;         ! DI14_ZS0104      - Fin de course : vérin étendu
    VAR signaldi boutonSoudureDI;     ! DI_Virtuel1_Bouton1 - bouton programmable 1

    ! VARIABLES POUR GESTION D'INTERRUPTION (TRAP)
    VAR intnum soudureInterrupt;       ! Un numero d'interruption non-utilise
    VAR bool soudureDemandee := FALSE; ! Flag pour indiquer qu'une soudure est en attente
    VAR bool lampeBleueActive := FALSE;! Flag pour l'etat de la lampe de proximite

! =====================================================================================
!   ROUTINE D'INTERRUPTION (TRAP)
! =====================================================================================
!-----------------------------------------------------------------------
! TRAP DemandeSoudure
! Description:
!   Cette routine est déclenchée par l'interruption 'soudureInterrupt'.
!   Elle met le drapeau 'soudureDemandee' à VRAI et met en pause
!   l'interruption jusqu'à ce qu'elle soit réactivée par IWatch.
!-----------------------------------------------------------------------
    TRAP DemandeSoudure
        soudureDemandee := TRUE;
        ! Suspendre la detection de cette interruption jusqu'a ce qu'elle soit reactivee
        ISleep soudureInterrupt;
        TPWrite "Soudure demandée! Sera exécutée après cette Opération.";
    ENDTRAP

! =====================================================================================
!   PROCEDURE PRINCIPALE
! =====================================================================================
!-----------------------------------------------------------------------
! PROC main
! Description:
!   Point d'entrée du programme. Elle initialise la cellule (I/O, zones),
!   vérifie la position de départ du robot, puis entre dans une boucle
!   infinie pour exécuter le cycle de manipulation des blocs.
!-----------------------------------------------------------------------
    PROC main()
        ! Initialisation de la cellule
        configIO; ! Mappage des alias I/O
        init;     ! Configurations initiales des zones et vitesses

        ! Configuration et activation de l'interruption pour la soudure
        CONNECT soudureInterrupt WITH DemandeSoudure;
        ISignalDI boutonSoudureDI, high, soudureInterrupt; ! Déclenche le TRAP sur front montant du bouton virtuel

        ! Verification de la position de depart du robot
        verifPositionAxes;

        ! Boucle principale du programme
        WHILE TRUE DO
            Deplacement_blocs;
        ENDWHILE
        ! Note: Le code place apres une boucle WHILE TRUE est unreachable et ne sera jamais execute.
    ENDPROC

! =====================================================================================
!   PROCEDURES D'INITIALISATION ET CONFIGURATION
! =====================================================================================

!-----------------------------------------------------------------------
! PROC configIO
! Description:
!   Mappe les signaux d'E/S physiques (hardware) à des variables
!   globales (alias) pour une utilisation plus intuitive dans le code.
!   Inclut une gestion d'erreur pour arrêter le programme si un alias
!   ne peut être créé.
!-----------------------------------------------------------------------
     PROC configIO()
        ! ==================== SORTIES (DO) ====================
        sigName:="DO01_EE_PINCE01"; AliasIO sigName,pinceFermer;
        sigName:="DO02_EE_VENT01";  AliasIO sigName,ventouse1Succion;
        sigName:="DO03_EE_VENT02";  AliasIO sigName,ventouse2Succion;
        sigName:="DO04_EE_LampBlu"; AliasIO sigName,lampeBleue;
        sigName:="DO05_EE_LampOr";  AliasIO sigName,lampeOrange;
        sigName:="DO09_FV0101";     AliasIO sigName,verinExtension;

        ! ==================== ENTRÉES (DI) ====================
        sigName:="DI01_EE_VAC01";   AliasIO sigName,objetSurVentouse1;
        sigName:="DI02_EE_VAC02";   AliasIO sigName,objetSurVentouse2;
        sigName:="DI09_ZS0101";     AliasIO sigName,blocPresentPoussoir;
        sigName:="DI10_ZS0102";     AliasIO sigName,blocOriente;
        sigName:="DI11_LSL0101";    AliasIO sigName,blocBasNiveau;
        sigName:="DI12_LSH0101";    AliasIO sigName,blocHautNiveau;
        sigName:="DI13_ZS0103";     AliasIO sigName,verinRetracte;
        sigName:="DI14_ZS0104";     AliasIO sigName,verinEtendu;
        sigName:="DI_Virtuel1_Bouton1"; AliasIO sigName,boutonSoudureDI;

        ! Si toutes les commandes AliasIO reussissent, la procedure se termine ici.
        RETURN;

    ERROR
        ! Ce bloc est execute si une des commandes AliasIO echoue.
        TPWrite "ERREUR: AliasIO a échoué pour le signal: " + sigName;
        TPWrite "Code d'erreur (ERRNO) = " \Num:=ERRNO;
        TPWrite "Programme arrêté.";
        EXIT; ! Arrêt du programme en cas d'échec
    ENDPROC

!-----------------------------------------------------------------------
! PROC init
! Description:
!   Configure l'environnement de travail du robot.
!   - Définit les limites articulaires pour la sécurité.
!   - Crée une zone de travail sphérique pour la détection de proximité.
!   - Initialise la vitesse, le vérin et la pince.
!-----------------------------------------------------------------------
    PROC init()
        ! Declaration des variables locales
        VAR shapedata joint_space;
        VAR jointtarget low_pos;
        VAR jointtarget high_pos;

        ! Definition des limites articulaires selon le type de controleur
        IF RobOS() THEN
            ! --- Limites pour le robot reel ---
            low_pos  := [[ -134, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
            high_pos := [[ -65, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
        ELSE
            ! --- Limites pour le controleur virtuel (RobotStudio) ---
            TPWrite "Utilisation de la cellule virtuelle dans Robostudio";
            low_pos  := [[ 44, -25, -85, -90, -103, -289 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
            high_pos := [[ 118, 55, 55, 90, 103, 109 ],[ 9E9, 9E9, 9E9, 9E9, 9E9, 9E9 ]];
        ENDIF

        ! --- Configuration de la zone de limitation articulaire ---
        WZFree LimArtic; ! Libere la zone si elle existe deja
        WZLimJointDef \Outside, joint_space, low_pos, high_pos; ! Definit les limites
        WZLimSup \Temp, LimArtic, joint_space; ! Active la supervision

        ! --- Configuration de la sphere de proximite ---
        WZFree ZoneProximite; ! Libere la zone si elle existe deja
        WZSphDef \Inside, sphProximite, rRetrait.trans, 150; ! Definit une sphere de 150mm de rayon
        WZDOSet \Temp, ZoneProximite \Inside, sphProximite, lampeBleue, 1; ! Allume la lampe bleue si le robot entre dans la sphere

        ! --- Initialisation des etats ---
        VelSet 25,1000; ! Limite la vitesse manuelle a 25% et la vitesse programmee a 1000 mm/s
        SetDO verinExtension, Retracte; ! S'assurer que le verin est retracte
        Pince\Ouvert; ! S'assurer que la pince est ouverte
    ENDPROC

!-----------------------------------------------------------------------
! PROC verifPositionAxes
! Description:
!   Vérifie si la position articulaire actuelle du robot correspond à la
!   position de départ 'rRetrait' dans une tolérance définie.
!   Affiche un message d'erreur si un ou plusieurs axes sont hors tolérance.
!-----------------------------------------------------------------------
    PROC verifPositionAxes()
        ! Declaration des variables locales
        VAR jointtarget posActuelle;        ! Angles actuels des axes J1-J6
        VAR jointtarget posReference;       ! Angles cibles calculés pour rRetrait
        VAR num delta;                      ! Écart angulaire pour chaque axe
        VAR bool erreurDetectee := FALSE;   ! True si un écart > seuil est détecté
        VAR string msgErreur := "Écart articulaire : "; ! Message d'erreur à afficher
        VAR num i;                          ! Compteur pour la boucle des axes
        VAR string tmp;                     ! Chaîne temporaire pour formater le message
        VAR num seuil;                      ! Tolérance angulaire (en degrés)

        ! Lecture de la configuration articulaire actuelle du robot
        posActuelle := CJointT();

        ! Calcul de la configuration articulaire de référence à partir du robtarget rRetrait
        posReference := CalcJointT(rRetrait, tPince_bloc\wobj:=wobj0);

        ! Boucle de comparaison pour chaque axe (J1 à J6)
        FOR i FROM 1 TO 6 DO
            ! Selectionne la tolerance (seuil) en fonction de l'axe
            IF i <= 3 THEN
                seuil := 1; ! Tolerance de 1 degre pour les axes majeurs (J1-J3)
            ELSE
                seuil := 5; ! Tolerance de 5 degres pour les axes du poignet (J4-J6)
            ENDIF

            ! Calcul de l'ecart absolu entre la position reelle et la reference
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
            ELSE
                delta := Abs(posActuelle.robax.rax_6 - posReference.robax.rax_6);
            ENDIF

            ! Si l'ecart depasse le seuil, on enregistre l'erreur
            IF delta > seuil THEN
                erreurDetectee := TRUE;
                tmp := "J" + NumToStr(i,0) + "(" + NumToStr(delta,1) + "°)";
                msgErreur := msgErreur + tmp; ! Ajoute l'info de l'axe au message
            ENDIF
        ENDFOR

        ! Affiche le resultat de la verification
        IF erreurDetectee THEN
            TPWrite msgErreur;
            TPWrite "Position initiale invalide. Programme arrêté.";
            EXIT; ! Stoppe l'exécution car la position de depart n'est pas sure
        ELSE
            TPWrite "Position initiale validée.";
        ENDIF
    ENDPROC

! =====================================================================================
!   PROCEDURES DE CYCLE (MOUVEMENTS ET ACTIONS)
! =====================================================================================

!-----------------------------------------------------------------------
! PROC Deplacement_blocs
! Description:
!   Orchestre le cycle complet de manipulation des blocs.
!   1. Prend 2 blocs de la glissoire et les empile.
!   2. Retourne à une position de sécurité.
!   3. Reprend les blocs de la pile et les redépose dans la glissoire.
!   Vérifie entre les actions si une soudure a été demandée.
!-----------------------------------------------------------------------
    PROC Deplacement_blocs()
        ! Declaration des variables locales
        VAR num bloc;
        VAR num num_blocs := 2;
        VAR num distance_offset;

        ! --- CYCLE 1: Prise a la glissoire et depot en pile ---
        FOR bloc FROM 0 TO (num_blocs - 1) DO
            ! Calcul de l'offset en Z pour empiler les blocs
            distance_offset := -((EpaisMM) * bloc);
            ! Calcule la position de depot pour le bloc courant
            rDepot_new := RelTool(rDepot, 0, 0, distance_offset);

            ! Execution des mouvements
            Prise_Glissoire;
            ManipulerBloc \Depot, rDepot_new; ! Utilise la nouvelle procedure unifiee

            ! Verifie si une soudure est demandee
            CheckAndRunSoudure;
        ENDFOR

        ! --- Aller a la position de retrait et attendre ---
        MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
        WaitTime 3;
        CheckAndRunSoudure; ! Verifie pour la soudure pendant la pause

        ! --- CYCLE 2: Prise de la pile et depot a la glissoire ---
        FOR bloc FROM (num_blocs - 1) TO 0 STEP -1 DO
            ! Calcul de l'offset en Z pour prendre le bloc du dessus de la pile
            distance_offset := -((EpaisMM) * bloc);
            ! Calcule la position de prise pour le bloc courant
            rDepot_new := RelTool(rDepot, 0, 0, distance_offset);

            ! Execution des mouvements
            ManipulerBloc \Prise, rDepot_new; ! Utilise la nouvelle procedure unifiee
            Depot_Glissoire;

            ! Verifie si une soudure est demandee
            CheckAndRunSoudure;
        ENDFOR

        ! --- Retour final a la position de retrait et attente ---
        MoveJ rRetrait, HighSpeed, fine, tPince_bloc\wobj:=wobj0;
        WaitTime 3;
        CheckAndRunSoudure; ! Verifie pour la soudure pendant la pause
    ENDPROC

!-----------------------------------------------------------------------
! PROC CheckAndRunSoudure
! Description:
!   Vérifie si le flag 'soudureDemandee' est vrai. Si c'est le cas,
!   elle appelle la procédure 'SimulerSoudure'.
!   Cette procédure centralise la logique de vérification.
!-----------------------------------------------------------------------
    PROC CheckAndRunSoudure()
        IF soudureDemandee = TRUE THEN
            SimulerSoudure;
        ENDIF
    ENDPROC

!-----------------------------------------------------------------------
! PROC Prise_Glissoire
! Description:
!   Gère la séquence de prise d'un bloc depuis la glissoire.
!   Attend un bloc si absent, actionne le vérin pour le positionner,
!   le saisit avec la pince, puis se rétracte.
!-----------------------------------------------------------------------
    PROC Prise_Glissoire()
        ! 1) Vérifier la présence d'un bloc dans la glissoire
        IF DInput(blocPresentPoussoir) = 0 THEN
            TPErase;
            TPWrite "Aucun bloc dans la glissoire";
            TPWrite "Placer deux blocs et remettre en mode AUTO";
            ! Si pas de bloc, attendre la détection
            WaitDI blocPresentPoussoir, 1;   ! Bloque jusqu'à ce qu'un bloc soit present
            WaitTime 1;                      ! Petite pause pour stabiliser le bloc
        ENDIF

        ! 2) Approche rapide vers la position de prise avec offset de sécurité
        MoveJ RelTool(rPriseGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
        ! 3) Indexer (avancer) le vérin pour caler le bloc contre la pince
        SetDO verinExtension, Extension;     ! Extension du vérin
        WaitDI verinEtendu, 1;               ! Attendre la fin de course

        ! 4) Descente linéaire pour position précise de la pince autour du bloc
        MoveL rPriseGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
        ! 5) Fermer la pince pour saisir le bloc
        Pince\Fermer;

        ! 6) Rentrer le vérin pour libérer la pièce et permettre le retrait
        SetDO verinExtension, Retracte;      ! Rétraction du vérin
        WaitDI verinRetracte, 1;             ! Attendre la fin de course de retraction

        ! 7) Retrait du robot vers position d'approche hors zone de collision
        MoveL RelTool(rPriseGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;
    ENDPROC

!-----------------------------------------------------------------------
! PROC Depot_Glissoire
! Description:
!   Gère la séquence de dépôt d'un bloc dans la glissoire.
!   Le robot s'approche, se positionne, ouvre la pince pour
!   relâcher le bloc, puis se rétracte.
!-----------------------------------------------------------------------
    PROC Depot_Glissoire()
        ! 1) Approche rapide vers la position de dépôt avec offset
        MoveJ RelTool(rDepotGli, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
        ! 2) Descente linéaire pour un positionnement précis devant la glissoire
        MoveL rDepotGli, LowSpeed, fine, tPince_bloc\wobj:=wobj0;

        ! 3) Ouvrir la pince pour libérer le bloc dans la glissoire
        Pince\Ouvert;

        ! 4) Reculer en mode linéaire avec offset pour libérer la zone
        MoveL RelTool(rDepotGli, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;
    ENDPROC

!-----------------------------------------------------------------------
! PROC ManipulerBloc
! Description:
!   Procédure générique pour prendre ou déposer un objet à une position
!   spécifiée. Remplace les anciennes procs `Depot` et `Prise_en_Depot`.
! Arguments:
!   \switch Prise: Ferme la pince pour saisir l'objet.
!   \switch Depot: Ouvre la pince pour relâcher l'objet.
!   robtarget rPos: La position cible pour l'action.
!-----------------------------------------------------------------------
    PROC ManipulerBloc(\switch Prise | switch Depot, robtarget rPos)
        ! 1) Approche rapide avec un offset de sécurité en Z
        MoveJ RelTool(rPos, 0, 0, Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;

        ! 2) Descente linéaire à vitesse réduite pour un positionnement précis
        MoveL rPos, LowSpeed, fine, tPince_bloc\wobj:=wobj0;

        ! 3) Actionne la pince selon le switch fourni
        IF Present(Prise) THEN
            Pince\Fermer;
        ELSE
            Pince\Ouvert;
        ENDIF

        ! 4) Remontée linéaire vers la position d'approche
        MoveL RelTool(rPos, 0, 0, Decalage), LowSpeed, z50, tPince_bloc\wobj:=wobj0;
    ENDPROC

! =====================================================================================
!   PROCEDURES UTILITAIRES (Pince, Crayon, Soudure)
! =====================================================================================

!-----------------------------------------------------------------------
! PROC Pince
! Description:
!   Contrôle l'ouverture et la fermeture de la pince. Vérifie l'état
!   actuel de la sortie pour éviter d'envoyer une commande redondante,
!   puis attend 1 seconde pour assurer l'actionnement mécanique.
! Arguments:
!   \switch Ouvert: Ouvre la pince.
!   \switch Fermer: Ferme la pince.
!-----------------------------------------------------------------------
    PROC Pince(\switch Ouvert | switch Fermer)
        ! Action sur la pince
        IF Present(Fermer) THEN
            ! Si la pince est actuellement ouverte (etat 0), on la ferme
            IF DOutput(pinceFermer) = Ouverte THEN
                SetDO pinceFermer, Fermee;
                WaitTime 1; ! Attendre le mouvement physique
            ENDIF
        ELSE
            ! Si la pince est actuellement fermee (etat 1), on l'ouvre
            IF DOutput(pinceFermer) = Fermee THEN
                SetDO pinceFermer, Ouverte;
                WaitTime 1; ! Attendre le mouvement physique
            ENDIF
        ENDIF
    ENDPROC

!-----------------------------------------------------------------------
! PROC LeCrayon
! Description:
!   Gère la prise ou la dépose du crayon à sa position dédiée.
! Arguments:
!   \switch Prise: Prend le crayon en fermant la pince.
!   \switch Deposer: Dépose le crayon en ouvrant la pince.
!-----------------------------------------------------------------------
    PROC LeCrayon(\switch Prise | switch Deposer)
        ! Mouvement d'approche vers le crayon
        MoveJ RelTool(rCrayon,0,0,Decalage), HighSpeed, z50, tPince_bloc\wobj:=wobj0;
        ! Mouvement precis vers la position du crayon
        MoveL rCrayon, LowSpeed, fine, tPince_bloc\wobj:=wobj0;
        WaitTime 0.5;

        ! Action sur la pince en fonction du parametre
        IF Present(Prise) THEN
            Pince\Fermer;
        ELSE
            Pince\Ouvert;
        ENDIF

        ! Mouvement de retrait
        MoveL RelTool(rCrayon,0,0,Decalage), LowSpeed, fine, tPince_bloc\wobj:=wobj0;
    ENDPROC

!-----------------------------------------------------------------------
! PROC SimulerSoudure
! Description:
!   Exécute une trajectoire de soudure simulée en forme de triangle.
!   - Calcule les points de la trajectoire.
!   - Prend un crayon pour simuler l'outil de soudure.
!   - Allume une lampe orange pendant le mouvement.
!   - Dépose le crayon et réactive l'interruption à la fin.
!-----------------------------------------------------------------------
    PROC SimulerSoudure()
        ! Declaration des variables locales
        VAR pos vecXY;
        VAR num vecNorm;
        VAR pos dirXY;
        VAR num transl_x;
        VAR num transl_y;
        VAR num angle_triangle_rad;
        VAR num angle_soudure_deg;
        VAR robtarget rSoudure_2_calc;
        VAR robtarget rSoud_3_temp;
        
        ! On utilise un délai de 0.3s) pour s'assurer qu'il s'active avant commencer et finir la simulation soudure.
        TriggIO trig_LightOn, 0.3 \Time \DOp:=lampeOrange, 1;
        TriggIO trig_LightOff, 0.3 \Time \DOp:=lampeOrange, 0;
    
        ! --- Etape 1: Preparation ---
        angle_triangle_rad := 30 * 3.14159265 / 180; ! Angle de 30 deg pour le triangle, converti en radians
        angle_soudure_deg := 30;                     ! Angle d'inclinaison de l'outil
        LeCrayon\Prise;                              ! Prendre le crayon

        ! --- Etape 2: Calcul de la trajectoire triangulaire ---

        ! Calcul du vecteur "vecXY" reliant rSoudure_1 à rSoudure_2
        vecXY := [rSoudure_2.trans.x - rSoudure_1.trans.x, rSoudure_2.trans.y - rSoudure_1.trans.y, rSoudure_2.trans.z - rSoudure_1.trans.z];

        ! Norme du vecteur vecXY (distance 3D entre point 1 et 2)
        vecNorm := Sqrt(Pow(vecXY.x,2) + Pow(vecXY.y,2) + Pow(vecXY.z,2));

        ! Vecteur unitaire "dirXY" (direction pure sans la magnitude)
        dirXY := [vecXY.x / vecNorm, vecXY.y / vecNorm, vecXY.z / vecNorm];

        ! Calcule 'rSoudure_2_calc' a 50 mm du point 1 dans la direction de point 2
        rSoudure_2_calc := rSoudure_1;
        rSoudure_2_calc.trans.x := rSoudure_1.trans.x + 50 * dirXY.x;
        rSoudure_2_calc.trans.y := rSoudure_1.trans.y + 50 * dirXY.y;
        rSoudure_2_calc.trans.z := rSoudure_1.trans.z + 50 * dirXY.z;

        ! Calcul du troisième point (rSoudure_3) pour former un triangle
        IF RobOS() THEN
            !--- Calcul pour le vrai robot ---
            transl_x := 50 * Cos(angle_triangle_rad); ! Projection en X
            transl_y := 50 * Sin(angle_triangle_rad); ! Projection en Y
            rSoud_3_temp := RelTool(rSoudure_2_calc, 0, 0, 0 \Rz := -60); ! Rotation de -60 deg
            rSoudure_3 := RelTool(rSoud_3_temp, transl_x, -transl_y, 0); ! Application de la translation
        ELSE
            !--- Calcul adapte pour le contrôleur virtuel (RobotStudio) ---
            angle_soudure_deg := 10;
            transl_x := 50 * Sin(angle_triangle_rad);
            transl_y := 50 * Cos(angle_triangle_rad);
            rSoud_3_temp := RelTool(rSoudure_2_calc, 0, 0, 0 \Rz := 60); ! Rotation de +60 deg
            rSoudure_3 := RelTool(rSoud_3_temp, transl_x, transl_y, 0);
        ENDIF
    
        ! --- Étape 3: Exécution de la trajectoire de soudure ---

        ! 1) Approche rapide de la position de départ.
        MoveJ RelTool(rSoudure_1,0,0,Decalage), HighSpeed, fine, tCrayon\WObj:=wobj0;
    
        ! 2) Préparer le trigger pour allumer la lampe au début de l'orientation.
        TriggL RelTool(rSoudure_1,0,0,0\Rx:=angle_soudure_deg), v100, trig_LightOn, fine, tCrayon\WObj:=wobj0;
    
        ! 3) Orientation précise de l'outil (la lampe s'allume via le trigger défini).
        MoveJ RelTool(rSoudure_1,0,0,0\Rx:=angle_soudure_deg), VeryLowSpeed, fine, tCrayon\WObj:=wobj0;
    
        ! 4) Exécution du contour triangulaire à vitesse très lente.
        MoveL RelTool(rSoudure_2_calc,0,0,0\Rx:=angle_soudure_deg), VeryLowSpeed, fine, tCrayon\WObj:=wobj0;
        MoveL RelTool(rSoudure_3,0,0,0\Rx:=angle_soudure_deg),     VeryLowSpeed, fine, tCrayon\WObj:=wobj0;
    
        ! 5) Préparer le trigger pour éteindre la lampe au début du retour.
        TriggL RelTool(rSoudure_1,0,0,0\Rx:=angle_soudure_deg), v100, trig_LightOff, fine, tCrayon\WObj:=wobj0;
    
        ! 6) Retour au point de départ (la lampe s'éteint via le trigger défini).
        MoveL RelTool(rSoudure_1,0,0,0\Rx:=angle_soudure_deg), VeryLowSpeed, fine, tCrayon\WObj:=wobj0;
    
        ! --- Etape 4: Finalisation ---
        WaitTime 0.5; ! Petite pause pour que la fin du cycle soit visible
        LeCrayon\Deposer;
        soudureDemandee := FALSE;
        IWatch soudureInterrupt;
    ENDPROC

ENDMODULE
