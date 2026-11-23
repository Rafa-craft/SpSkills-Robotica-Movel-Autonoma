/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants 
{
    public static final class DriveConstants 
    {
        // Configuração dos motores no controlador TitanQuad (Studica)
        public static final int kTitanCanId = 42;
        public static final int kMotor1Channel = 2; // Traseira
        public static final int kMotor2Channel = 3; // Frontal Direita
        public static final int kMotor3Channel = 1; // Frontal Esquerda

        // Canais dos encoders no TitanQuad (mesmo indice do motor)
        public static final int kEncoder1Channel = kMotor1Channel;
        public static final int kEncoder2Channel = kMotor2Channel;
        public static final int kEncoder3Channel = kMotor3Channel;

        // CONFERIR NO HARDWARE: pulsos por volta da roda (Maverick + caixa de reducao)
        public static final double kEncoderTicksPerRevolution = 1440.0;
        // CONFERIR NO HARDWARE: diametro da roda omni, em metros
        public static final double kWheelDiameterMeters = 0.110;
        // Metros percorridos por pulso do encoder
        public static final double kEncoderDistancePerTick =
                (Math.PI * kWheelDiameterMeters) / kEncoderTicksPerRevolution;

        // CONFERIR NO HARDWARE: distancia do centro do robo ao centro de cada
        // roda, em metros. So entra na odometria, para converter o arco de
        // rotacao medido pelas rodas em radianos.
        public static final double kRobotRadiusMeters = 0.150;

        // Sinal da IMU: false = getAngle() ja e anti-horario positivo, que e a
        // convencao da WPILib. Valor confirmado no robo - e o que faz o
        // field-oriented apontar certo. Esta flag vale para a odometria E para
        // o driveFieldOriented ao mesmo tempo, entao os dois nunca divergem.
        public static final boolean kGyroReversed = false;

        // Limite nominal do motor DC Maverick com caixa de redução (RPM)
        public static final double kMaxRPM = 100.0;

        // Ganhos do PID de Velocidade das rodas
        public static final double kP = 0.001;
        public static final double kI = 0.000;
        public static final double kD = 0.000;

        // Limites do SlewRateLimiter (Variação máxima de potência por segundo)
        public static final double kSlewRateX = 2.0;
        public static final double kSlewRateY = 2.0;
        public static final double kSlewRateZ = 3.0;
        
        // Constantes de ganho do feedForward
        public static final double kS = 0.03; 
        public static final double kV = 0.1; 
    }

    public static final class AutoConstants 
    {
        // Lado do quadrado, em metros
        public static final double kSquareSideMeters = 0.50;

        // Velocidade de cruzeiro (fracao de kMaxRPM)
        public static final double kDriveSpeed = 0.35;
        // Piso de velocidade: abaixo disso o robo nao vence o atrito
        public static final double kMinDriveSpeed = 0.12;
        // Distancia final em que o robo comeca a desacelerar (metros)
        public static final double kRampDownDistance = 0.15;
        // Quanto antes do alvo o lado e dado por concluido (metros)
        public static final double kDistanceTolerance = 0.01;

        // Correcao proporcional de rumo durante a translacao (saida por grau de erro).
        // TESTE COM 0.0 NA PRIMEIRA VEZ: se o robo girar em vez de corrigir,
        // o sinal esta invertido - use -0.010.
        public static final double kHeadingP = 0.010;
        // Teto da correcao de rumo, para nao virar uma rotacao
        public static final double kMaxTurnOutput = 0.15;

        // Rede de seguranca: cada lado desiste depois disso (segundos)
        public static final double kLegTimeoutSeconds = 6.0;
        // Pausa entre os lados, para o robo assentar antes de trocar de direcao
        public static final double kPauseSeconds = 0.25;

        // --- Navegacao por coordenadas de campo (DriveToPointCommand) ---
        // Saida de velocidade por metro de erro. Com 0.8 o robo sai na
        // velocidade de cruzeiro enquanto falta mais de ~0.45 m e desacelera
        // sozinho no fim, sem precisar da rampa manual do DriveDistanceCommand.
        public static final double kTranslationP = 0.8;
        // Raio em volta do ponto em que ele e considerado alcancado (metros).
        // Nao aperte abaixo da precisao real da odometria, senao o comando fica
        // oscilando em volta do alvo ate o timeout.
        public static final double kPositionTolerance = 0.05;
        // Parada em cada ponto do trajeto (segundos)
        public static final double kWaypointPauseSeconds = 1.0;
        // Rede de seguranca por ponto: cada perna desiste depois disso (segundos)
        public static final double kWaypointTimeoutSeconds = 8.0;
    }

    /**
     * Arena com os pontos A-D. As coordenadas (metros) estao no MESMO referencial da
     * odometria e do DriveToPointCommand: sao os mesmos eixos de pose.getTranslation().
     */
    public static final class ArenaConstants 
    {
        // Pose do robo quando o autonomo comeca (a odometria e ancorada aqui)
        public static final double kStartX = 0.0;
        public static final double kStartY = 0.0;
        public static final double kStartHeadingDegrees = 0.0;

        // Pontos de destino. VALORES HIPOTETICOS: troque pelos pontos reais da arena.
        // Mantenha cada ponto mais longe das paredes que LidarConstants.kSafeDistanceMeters.
        public static final double kPointAX = 0.00;
        public static final double kPointAY = 0.00;
        public static final double kPointBX = 0.50;
        public static final double kPointBY = 0.00;
        public static final double kPointCX = 0.50;
        public static final double kPointCY = 0.50;
        public static final double kPointDX = 0.00;
        public static final double kPointDY = 0.50;
    }

    public static final class LidarConstants 
    {
        // CONFERIR NO ROBO: unidade que getDistanceAt() devolve.
        // 0.001 se vier em milimetros, 1.0 se vier em metros. Veja a telemetria
        // "Lidar/Dist ... (m)": com uma parede a 0,5 m ela deve marcar ~0.5.
        public static final double kDistanceToMeters = 1.0  ;

        // CONFERIR NO ROBO: angulos do LiDAR (graus, a partir do zero do sensor) que
        // apontam para +x e para +y da odometria (os eixos de driveRobotOriented).
        //   +x = direita do robo, +y = frente  -> 90.0 e   0.0 (padrao, igual ao
        //                                         comentario do seu LidarSubsystem)
        //   +x = frente, +y = esquerda         ->  0.0 e 270.0
        // Para saber qual e o seu caso, mande o robo andar para +x e veja para onde ele vai.
        public static final double kAnglePlusXDegrees = 90.0;
        public static final double kAnglePlusYDegrees = 0.0;
        // true se o angulo do LiDAR cresce de +x para +y (calculado, nao mexa)
        public static final boolean kYFollowsX =
                ((kAnglePlusYDegrees - kAnglePlusXDegrees + 360.0) % 360.0) == 90.0;

        // Varredura usada no desvio: um raio a cada kRayStepDegrees, com tolerancia
        public static final int kRayStepDegrees = 20;
        public static final int kRayToleranceDegrees = 8;
        // Leituras menores que isso (metros) sao ignoradas: invalidas ou corpo do robo
        public static final double kMinValidMeters = 0.10;

        // Distancia do LiDAR (metros) a partir da qual o robo comeca a se afastar.
        // Lembre que o LiDAR mede do centro do robo: some o raio do chassi.
        public static final double kSafeDistanceMeters = 0.42;
        // Forca da repulsao (1.0 = no encosto ela iguala a velocidade de cruzeiro; 0.0 desliga o desvio)
        public static final double kAvoidGain = 1.0;
        // Forca do empurrao lateral quando o obstaculo esta de frente
        public static final double kSidestepGain = 1.0;
        // Lateral abaixo disso e considerada "obstaculo centrado"
        public static final double kSideDeadband = 0.05;
        // Lado do desvio quando o obstaculo esta centrado: +1 ou -1
        public static final double kDefaultDodgeSide = 1.0;
        // Dentro dessa distancia (metros) do alvo o desvio e reduzido ate zero
        public static final double kAvoidFadeDistanceMeters = 0.30;
    }

    public static final class OIConstants 
    {
        public static final int kJoystickPort = 0;
        public static final double kDeadband = 0.05; // 5% de zona morta
        public static final int kResetGyroButton = 1; // Botão Trigger para reset do campo
    }
}