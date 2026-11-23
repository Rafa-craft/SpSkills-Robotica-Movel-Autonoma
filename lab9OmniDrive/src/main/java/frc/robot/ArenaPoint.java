package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;

import frc.robot.Constants.ArenaConstants;

/**
 * Pontos de destino da arena. As coordenadas (em metros) ficam em
 * {@link ArenaConstants}; aqui so se da nome a cada uma.
 */
public enum ArenaPoint
{
    A("A", ArenaConstants.kPointAX, ArenaConstants.kPointAY),
    B("B", ArenaConstants.kPointBX, ArenaConstants.kPointBY),
    C("C", ArenaConstants.kPointCX, ArenaConstants.kPointCY),
    D("D", ArenaConstants.kPointDX, ArenaConstants.kPointDY);

    private final String label;
    private final Translation2d position;

    ArenaPoint(String label, double xMeters, double yMeters)
    {
        this.label = label;
        this.position = new Translation2d(xMeters, yMeters);
    }

    /** @return nome do ponto ("A" a "D") */
    public String getLabel()
    {
        return label;
    }

    /** @return posicao do ponto no referencial da arena/odometria, em metros */
    public Translation2d getPosition()
    {
        return position;
    }
}