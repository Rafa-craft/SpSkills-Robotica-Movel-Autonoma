package frc.robot.commands;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.ArenaPoint;
import frc.robot.Constants.ArenaConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.LidarSubsystem;
import frc.robot.subsystems.OmnidriveSubsystem;

/**
 * Autonomo: sorteia a ordem dos pontos A, B, C e D da arena e leva o robo ate
 * cada um, nessa ordem, desviando de obstaculos com o LiDAR.
 *
 * <p>O sorteio acontece em {@link #initialize()}, a cada vez que o autonomo
 * comeca. Um SequentialCommandGroup montado no construtor (como o
 * WaypointAutoCommand) sortearia uma unica vez, quando o robo liga. Como a
 * WPILib 2020 nao tem DeferredCommand, este comando monta o grupo na hora e
 * repassa initialize/execute/isFinished/end para ele.
 *
 * <p>Cada ponto sorteado aparece uma unica vez (permutacao de A-D). A rota e o
 * destino atual vao para o SmartDashboard em "Auto/Rota sorteada" e
 * "Auto/Destino atual".
 *
 * <p>Sequencia de cada rota: define a pose inicial, e para cada ponto vai ate
 * ele (com timeout de seguranca) e para por {@link AutoConstants#kWaypointPauseSeconds}.
 */
public class RandomPointsAutoCommand extends CommandBase
{
    private final OmnidriveSubsystem drive;
    private final LidarSubsystem lidar;
    private final Random random = new Random();

    /** Grupo montado a cada initialize(). */
    private Command route = new InstantCommand();

    public RandomPointsAutoCommand(OmnidriveSubsystem drive, LidarSubsystem lidar)
    {
        this.drive = drive;
        this.lidar = lidar;

        addRequirements(drive, lidar);
    }

    @Override
    public void initialize()
    {
        List<ArenaPoint> order = new ArrayList<>(Arrays.asList(ArenaPoint.values()));
        Collections.shuffle(order, random);

        SmartDashboard.putString("Auto/Rota sorteada", describe(order));

        SequentialCommandGroup group = new SequentialCommandGroup(
            // Ancora a odometria na pose inicial da arena e liga o LiDAR
            new InstantCommand(() ->
            {
                lidar.startScan();
                drive.resetPose(startPose());
            })
        );

        for (ArenaPoint point : order)
        {
            group.addCommands(
                new InstantCommand(() -> SmartDashboard.putString("Auto/Destino atual", point.getLabel())),
                leg(point),
                pause()
            );
        }

        group.addCommands(
            new InstantCommand(() -> SmartDashboard.putString("Auto/Destino atual", "concluido")),
            new InstantCommand(drive::stop, drive)
        );

        route = group;
        route.initialize();
    }

    @Override
    public void execute()
    {
        route.execute();
    }

    @Override
    public boolean isFinished()
    {
        return route.isFinished();
    }

    @Override
    public void end(boolean interrupted)
    {
        route.end(interrupted);
        drive.stop();
    }

    private static Pose2d startPose()
    {
        return new Pose2d(ArenaConstants.kStartX,
                          ArenaConstants.kStartY,
                          Rotation2d.fromDegrees(ArenaConstants.kStartHeadingDegrees));
    }

    /** Uma perna do trajeto, com a rede de seguranca de tempo. */
    private Command leg(ArenaPoint point)
    {
        return new DriveToPointAvoidCommand(drive, lidar, point.getPosition())
                .withTimeout(AutoConstants.kWaypointTimeoutSeconds);
    }

    /** Parada em cima do ponto, segurando a tracao (veja o comentario no WaypointAutoCommand). */
    private Command pause()
    {
        return new RunCommand(drive::stop, drive)
                .withTimeout(AutoConstants.kWaypointPauseSeconds);
    }

    private static String describe(List<ArenaPoint> order)
    {
        StringBuilder text = new StringBuilder();
        for (int i = 0; i < order.size(); i++)
        {
            if (i > 0)
            {
                text.append(" -> ");
            }
            text.append(order.get(i).getLabel());
        }
        return text.toString();
    }
}