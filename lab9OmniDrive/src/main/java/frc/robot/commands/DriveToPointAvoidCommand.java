package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.LidarConstants;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.OmnidriveSubsystem;

/**
 * Igual ao {@link DriveToPointCommand} (vai ate um ponto absoluto usando a
 * odometria, sem girar o chassi), mas desviando de obstaculos com o LiDAR.
 *
 * <p>A velocidade final e a soma de dois vetores, ambos no referencial do campo:
 * <ul>
 *   <li>atracao: aponta para o alvo, com velocidade proporcional a distancia;</li>
 *   <li>repulsao: vem do {@link LidarSubsystem#getRepulsionVector(double)} (referencial
 *       do robo) e e girada pela pose para o campo. Empurra para longe de tudo que
 *       esta mais perto que {@link LidarConstants#kSafeDistanceMeters}.</li>
 * </ul>
 *
 * <p>Dois ajustes evitam os problemas classicos desse metodo:
 * <ul>
 *   <li>obstaculo exatamente de frente: a repulsao anularia a atracao e o robo
 *       travaria. Entao soma-se um empurrao lateral, para o lado em que o
 *       obstaculo ja "pede" desvio (ou o lado padrao, se estiver centrado);</li>
 *   <li>perto do alvo o desvio vai a zero, senao um ponto proximo da parede
 *       nunca seria alcancado.</li>
 * </ul>
 *
 * <p>Se o robo ainda assim ficar preso, o timeout aplicado por quem chama (veja
 * {@code RandomPointsAutoCommand}) passa para o ponto seguinte.
 */

public class DriveToPointAvoidCommand extends CommandBase
{
    private final OmnidriveSubsystem drive;
    private final LidarSubsystem lidar;

    /** Ponto de destino no referencial do campo, em metros. */
    private final Translation2d target;

    /** Rumo a manter durante o trajeto, capturado no initialize(). */
    private Rotation2d targetHeading = new Rotation2d();

    /** Lado do desvio (+1 ou -1) relativo ao eixo perpendicular a direcao do alvo. */
    private double dodgeSide = LidarConstants.kDefaultDodgeSide;

    /**
     * @param drive   subsistema de tracao
     * @param lidar   subsistema do LiDAR (precisa estar escaneando)
     * @param target  ponto de destino, em metros
     */
    public DriveToPointAvoidCommand(OmnidriveSubsystem drive, LidarSubsystem lidar, Translation2d target)
    {
        this.drive = drive;
        this.lidar = lidar;
        this.target = target;

        addRequirements(drive);
    }

    @Override
    public void initialize()
    {
        targetHeading = drive.getPose().getRotation();
        dodgeSide = LidarConstants.kDefaultDodgeSide;
        lidar.startScan();
    }

    @Override
    public void execute()
    {
        Pose2d pose = drive.getPose();
        Translation2d error = target.minus(pose.getTranslation());
        double distance = error.getNorm();

        if (distance < 1e-6)
        {
            drive.driveFieldOriented(0.0, 0.0, 0.0);
            return;
        }

        // Atracao: mesma lei do DriveToPointCommand
        double speed = Math.min(distance * AutoConstants.kTranslationP, AutoConstants.kDriveSpeed);
        speed = Math.max(speed, AutoConstants.kMinDriveSpeed);

        Translation2d goalDir = error.div(distance);
        Translation2d velocity = goalDir.times(speed);

        // Repulsao: sai no referencial do robo, entao gira para o campo
        Translation2d repulsion = lidar.getRepulsionVector(LidarConstants.kSafeDistanceMeters)
                                       .rotateBy(pose.getRotation());

        // Perto do alvo o desvio some, para o robo conseguir terminar o ponto
        double fade = Math.min(1.0, distance / LidarConstants.kAvoidFadeDistanceMeters);
        double avoidScale = LidarConstants.kAvoidGain * fade * AutoConstants.kDriveSpeed;
        velocity = velocity.plus(computeAvoidance(goalDir, repulsion).times(avoidScale));

        // Nunca passa da velocidade de cruzeiro
        double norm = velocity.getNorm();
        if (norm > AutoConstants.kDriveSpeed)
        {
            velocity = velocity.times(AutoConstants.kDriveSpeed / norm);
        }

        // Correcao de rumo: o robo translada, nao gira (igual ao DriveToPointCommand)
        double headingError = targetHeading.minus(pose.getRotation()).getDegrees();
        double turn = headingError * AutoConstants.kHeadingP;
        turn = Math.max(-AutoConstants.kMaxTurnOutput,
               Math.min(AutoConstants.kMaxTurnOutput, turn));

        drive.driveFieldOriented(velocity.getX(), velocity.getY(), turn);
    }

    @Override
    public boolean isFinished()
    {
        return target.minus(drive.getPose().getTranslation()).getNorm()
                <= AutoConstants.kPositionTolerance;
    }

    @Override
    public void end(boolean interrupted)
    {
        drive.stop();
    }

    /**
     * Repulsao do LiDAR mais o empurrao lateral que contorna obstaculos de frente.
     *
     * @param goalDir   direcao unitaria do alvo, no campo
     * @param repulsion repulsao do LiDAR, no campo (aponta para longe do obstaculo)
     * @return vetor de desvio, no campo
     */
    private Translation2d computeAvoidance(Translation2d goalDir, Translation2d repulsion)
    {
        // perpendicular a direcao do alvo (goalDir girado +90 graus)
        Translation2d perp = new Translation2d(-goalDir.getY(), goalDir.getX());

        // componente da repulsao ao longo do alvo (negativa = obstaculo na frente)
        double along = repulsion.getX() * goalDir.getX() + repulsion.getY() * goalDir.getY();
        // componente lateral (para que lado o obstaculo ja esta empurrando)
        double side = repulsion.getX() * perp.getX() + repulsion.getY() * perp.getY();

        if (Math.abs(side) > LidarConstants.kSideDeadband)
        {
            dodgeSide = Math.signum(side);
        }

        double ahead = Math.max(0.0, -along);
        return repulsion.plus(perp.times(dodgeSide * ahead * LidarConstants.kSidestepGain));
    }
}