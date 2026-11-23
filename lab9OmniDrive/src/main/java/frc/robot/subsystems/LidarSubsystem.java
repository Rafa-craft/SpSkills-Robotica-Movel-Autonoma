package frc.robot.subsystems;

import com.studica.frc.LidarMini;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LidarConstants;

public class LidarSubsystem extends SubsystemBase {
  
  private final LidarMini lidarMini;
  public LidarMini.ScanData scanData;
  public boolean scanning = false;

  // Controla qual ângulo está sendo lido no ciclo atual (de 0 a 355 graus)
  private int currentScanAngle = 0;

  public LidarSubsystem() {
    lidarMini = new LidarMini(LidarMini.Port.kUSB1);
  }

  public void startScan() {
    if (!scanning) {
      lidarMini.start();
      scanning = true;
    }
  }

  public void stopScan() {
    if (scanning) {
      lidarMini.stop();
      scanning = false;
    }
  }

  /**
 * Calcula a correção de giro (wz) para manter a lateral direita (90°)
 * perfeitamente perpendicular à parede da arena.
 *
 * @return velocidade angular wz (-kMaxTurnOutput a +kMaxTurnOutput)
 */
public double getWallAlignmentTurnOutput() {
  if (!scanning) return 0.0;
  
  scanData = lidarMini.getData();
  if (scanData == null) return 0.0;

  // Lê três pontos na lateral direita: 75° (frente-direita), 90° (meio) e 105° (trás-direita)
  float d75 = scanData.getDistanceAt(75, 2);
  float d90 = scanData.getDistanceAt(90, 2);
  float d105 = scanData.getDistanceAt(105, 2);

  // Proteção: se algum raio falhar (<= 0) ou estiver muito longe (fora da parede), não gira
  if (d75 <= 0.05f || d90 <= 0.05f || d105 <= 0.05f) {
    return 0.0;
  }

  // Checagem de quina ou obstáculo irregular:
  // Em uma parede reta, d90 DEVE ser menor que a média de d75 e d105.
  // Se d90 for maior, o LiDAR está olhando para uma quina externa ou buraco.
  float mediaPontas = (d75 + d105) / 2.0f;
  if (d90 > mediaPontas) {
    return 0.0; // Ignora a leitura para o robô não girar loucamente numa quina
  }

  // Erro de simetria: se d75 == d105, erro é zero (robô perpendicular)
  double erro = d105 - d75;

  // Ganho proporcional (ajuste conforme a unidade do seu LiDAR:
  // Se o LiDAR retorna em mm, use algo como 0.002. Se for em metros, use 2.0)
  double kWallAlignP = 0.002; 
  double turn = erro * kWallAlignP;

  // Limita a velocidade máxima de giro para não desestabilizar o movimento
  double maxTurn = 0.15;
  return Math.max(-maxTurn, Math.min(maxTurn, turn));
}


  public void putDataLidar() {
    if (!scanning) return;
    scanData = lidarMini.getData();
    if (scanData == null) return;

  
    float d = scanData.getDistanceAt(90, 2);
    // Arredonda para 2 casas decimais no SmartDashboard
    double distLimpa = Math.round(d * 100.0) / 100.0;
    SmartDashboard.putNumber("LidarSubsystem/ Distance limpa 90deg", distLimpa);
    for(int i =179; i <= 360; i++){
      
      SmartDashboard.putNumber("LidarSubsystem/ Distance []" + i + "Degree", scanData.getDistanceAt(i, 1));

    }
  }
  public double alingmentToWall(int angulo)
  {
    if (!scanning) return 0.0;
  
    scanData = lidarMini.getData();
    if (scanData == null) return 0.0;
    double menorDistancia = scanData.getDistanceAt(angulo - 30, 3);
    int menorAngulo = 0;
    int correcao = 0;

    for(int i = angulo -30; angulo >= angulo + 30; i+=6)
    {
      if(menorDistancia > scanData.getDistanceAt(i, 3) )
      {
        menorDistancia = scanData.getDistanceAt(i, 3);
        menorAngulo = i;
      }
    }

    if(menorAngulo != 270)
    {
      correcao = menorAngulo - 270;
    } else {
      return 0;
    }
    
    return correcao;
    
  }

  @Override
  public void periodic ()
  {
    publishCalibrationData();

  }

  /**
   * Vetor de repulsão no referencial do ROBÔ (mesmos eixos de driveRobotOriented), apontando
   * para longe dos obstáculos mais perto que safeDistanceMeters. O módulo vai de 0 (nada por
   * perto) até 1 (encostado). Retorna zero se o LiDAR não estiver escaneando.
   *
   * <p>Corrige a versão anterior: ela somava 90 (graus) a um ângulo em radianos e dependia de
   * adivinhar a unidade do sensor. Agora a unidade e a orientação vêm de {@link LidarConstants}.
   *
   * @param safeDistanceMeters distância (m) abaixo da qual um obstáculo empurra o robô
   * @return vetor de repulsão, módulo de 0 a 1
   */
  public Translation2d getRepulsionVector(double safeDistanceMeters) {
    if (!scanning) {
      return new Translation2d();
    }
    scanData = lidarMini.getData();
    if (scanData == null) {
      return new Translation2d();
    }

    double sumX = 0.0;
    double sumY = 0.0;
    int rays = 0;

    for (int angle = 0; angle < 360; angle += LidarConstants.kRayStepDegrees) {
      rays++;
      double distance = getDistanceMeters(scanData, angle, LidarConstants.kRayToleranceDegrees);

      // Leitura inválida, colada no sensor (corpo do robô) ou fora da zona de segurança
      if (distance < LidarConstants.kMinValidMeters || distance >= safeDistanceMeters) {
        continue;
      }

      // 0 na borda da zona de segurança, 1 encostado
      double strength = (safeDistanceMeters - distance) / safeDistanceMeters;
      Translation2d toObstacle = lidarAngleToRobotDirection(angle);

      // Subtrai: o empurrão é na direção OPOSTA ao obstáculo
      sumX -= toObstacle.getX() * strength;
      sumY -= toObstacle.getY() * strength;
    }

    // Normaliza pelo número de raios (4/rays faz uma parede reta encostada chegar perto de 1)
    Translation2d repulsion = new Translation2d(sumX, sumY).times(4.0 / rays);
    double norm = repulsion.getNorm();
    return (norm > 1.0) ? repulsion.div(norm) : repulsion;
  }

  /** Distância em metros no ângulo pedido; 0.0 quando a leitura é inválida. */
  private static double getDistanceMeters(
      LidarMini.ScanData data, int lidarAngleDegrees, int toleranceDegrees) {
    float raw = data.getDistanceAt(lidarAngleDegrees, toleranceDegrees);
    return (raw > 0.0f) ? raw * LidarConstants.kDistanceToMeters : 0.0;
  }

  /**
   * Converte um ângulo do LiDAR (graus) na direção correspondente no referencial do robô.
   * Os eixos +X e +Y da odometria apontam para kAnglePlusXDegrees e kAnglePlusYDegrees.
   */
  private static Translation2d lidarAngleToRobotDirection(double lidarAngleDegrees) {
    double fromPlusX = lidarAngleDegrees - LidarConstants.kAnglePlusXDegrees;
    double psi = Math.toRadians(LidarConstants.kYFollowsX ? fromPlusX : -fromPlusX);
    return new Translation2d(Math.cos(psi), Math.sin(psi));
  }

  /** Telemetria para conferir unidade e orientação do LiDAR (veja LidarConstants). */
  private void publishCalibrationData() {
    if (!scanning) {
      return;
    }
    scanData = lidarMini.getData();
    if (scanData == null) {
      return;
    }
    SmartDashboard.putNumber("Lidar/Dist 0 graus (m)", getDistanceMeters(scanData, 0, 3));
    SmartDashboard.putNumber("Lidar/Dist 90 graus (m)", getDistanceMeters(scanData, 90, 3));
    SmartDashboard.putNumber("Lidar/Dist 180 graus (m)", getDistanceMeters(scanData, 180, 3));
    SmartDashboard.putNumber("Lidar/Dist 270 graus (m)", getDistanceMeters(scanData, 270, 3));
  }

}