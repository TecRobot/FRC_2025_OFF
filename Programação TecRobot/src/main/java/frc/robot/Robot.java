// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// Importação das bibliotecas
import com.ctre.phoenix.motorcontrol.IFollower;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.AnalogJNI.AnalogTriggerType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;


// Tem que ser essas bibliotecas para fazer o shufleboard com autonomos selecionado
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.net.PortForwarder;







/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> auto_Chooser = new SendableChooser<>();
  private String selectedAuto;
  
  // Declaração do chassi da tração
  DifferentialDrive chassi;
  // Declaração dos motores Victor e Spark
  public MotorController motorEsquerdaMestre= new WPI_VictorSPX(4);
  public MotorController motorEsquerda= new WPI_VictorSPX(1);
  public MotorController motorDireitaMestre= new WPI_VictorSPX(3);
  public MotorController motorDireita= new WPI_VictorSPX(2);
  public MotorController rampa = new WPI_VictorSPX(6);
  public SparkMax agarrador = new SparkMax(3, MotorType.kBrushless);
  public SparkMax garra = new SparkMax(2, MotorType.kBrushless);
  // Declaração do controle
  XboxController controle = new XboxController(0);
  // Declaração do tempo
  double startTime;
  
  // Declaração dos sensores
  private AnalogInput sharpSensor1;
  private AnalogInput sharpSensor2;
  
  private double[] sensorValue1;
  private double[] sensorValue2;
  private static final int NUM_VALUES = 100;
  private int index;

  private PhotonCamera camera;
  
    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    public Robot() {
      sharpSensor1 = new AnalogInput(0);
      sharpSensor2 = new AnalogInput(1);
      sensorValue1 = new double[NUM_VALUES];
      sensorValue2 = new double[NUM_VALUES];
      // Colocando os motores direita e esquerda seguirem os seus motores mestre
      ((IFollower) motorEsquerda).follow((IMotorController) motorEsquerdaMestre);
      ((IFollower) motorDireita).follow((IMotorController) motorDireitaMestre);
      // Invertendo os motores da direita
      motorDireitaMestre.setInverted(true);
      motorDireita.setInverted(true);
      // Colocando dentro do chassi quem é o motor da esquerda e o da direita
      chassi = new DifferentialDrive(motorEsquerdaMestre, motorDireitaMestre);
      
      // Configurando o motor neo da garra para ter o modo brake ativado
      SparkMaxConfig globalConfig = new SparkMaxConfig();
      globalConfig
              .idleMode(IdleMode.kBrake);
      garra.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      
      //declaração da câmera
      CameraServer.startAutomaticCapture();
      CameraServer.startAutomaticCapture();
      
      auto_Chooser.setDefaultOption("AutonomoEsquerda", "Esquerda");
      auto_Chooser.setDefaultOption("AutonomoDireita", "Direita");
      auto_Chooser.setDefaultOption("AutonomoMeio", "Meio");

      ShuffleboardTab tab = Shuffleboard.getTab("Autônomos");
      tab.add("Escolher Autônomo", auto_Chooser);

      // Câmera
      
      PortForwarder.add(5800, "localhost", 5800);
      

      /* 
      m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
      m_chooser.addOption("My Auto", kCustomAuto);
      SmartDashboard.putData("Auto choices", m_chooser);*/
    }
    
    
              
                /**
     * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
    }
  
    /**
     * This autonomous (along with the chooser code above) shows how to select between different
     * autonomous modes using the dashboard. The sendable chooser code works with the Java
     * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
     * uncomment the getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to the switch structure
     * below with additional strings. If using the SendableChooser make sure to add them to the
     * chooser code above as well.
     */
    @Override
    public void autonomousInit() {
      selectedAuto = auto_Chooser.getSelected();
      System.out.println("Modo autônomo selecionado:" + selectedAuto);
     
      /*m_autoSelected = m_chooser.getSelected();
      System.out.println("Auto selected: " + m_autoSelected); */
     
      // Iniciando o timer do autonomus
      startTime = Timer.getFPGATimestamp();
      //sensor
      index = 0;

    }
  
    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {

      double time = Timer.getFPGATimestamp();
      /* essa lógica serve para selecionar de qual posição o robo partirá no modo Autonomus */
      switch (selectedAuto) {
        case "Meio":
          Meio(time);
          break;
        case "Esquerda":
          Esquerda(time);
          break;
        case "Direita":
          Direita(time);
          break;
        default:
          break;  
      }
    }
    // Esse é o autonomus que será utilizado caso o robô comece na direita
     private void Direita(double time) {
      // Aqui fazemos uma média dos valores recebidos pelos sensores, cada um por segundo, 
      // já que ele retorna vários valores e não dar problema na hora
      double currentValue1 = sharpSensor1.getValue();
      double currentValue2 = sharpSensor2.getValue();
      sensorValue1[index] = currentValue1;
      sensorValue2[index] = currentValue2;
      index = (index + 1) % NUM_VALUES;
      double sum = 0.0;
    for (double value : sensorValue1) {
        sum += value;
    }
    double average1 = sum / NUM_VALUES;

    for (double value : sensorValue2) {
      sum += value;
  }
  double average2 = sum / NUM_VALUES;

    // Envie a média para o SmartDashboard
    SmartDashboard.putNumber("Sharp Sensor Average1", average1);
    SmartDashboard.putNumber("Sharp Sensor Average2", average2);
      
      // Definir o tempo inicial e limite dessa parte dos if
      if(time - startTime < 2.5) {
        motorDireitaMestre.set(0.3385); 
        motorEsquerdaMestre.set(0.3);
      }
      else {
        motorDireitaMestre.set(0); 
        motorEsquerdaMestre.set(0);
      }  
        /* 
        // Aqui é definido a velocidade do robô para ele ir para frente
        motorDireitaMestre.set(0.3385);
        motorEsquerdaMestre.set(0.30);
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 3 && time - startTime < 3.5) {
        // Aqui é definido a velocidade do robô para ele girar no próprio eixo, para a direita
        motorDireitaMestre.set(0.3385);
        motorEsquerdaMestre.set(-0.30);
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 3.5 && time - startTime < 5) {
        // Aqui é definido a velocidade do robô para ele ir para frente
        motorDireitaMestre.set(0.3385);
        motorEsquerdaMestre.set(0.30);
          
          
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 5.5 && time - startTime < 7.5) {
        // Aqui definimos a distância que o robô deve está do recife, para pode soltar o coral no recife
        if(average1 > 500 && average1 < 1600 && average2 > 500 && average2 < 1600) {
          // Aqui definimos a velocidade do motor da rampa, para soltar o coral
           rampa.set(-0.37);
            }
          else {
          // Definir a velocidade do motor para zero
           rampa.set(0);
          }
        // Quando nenhuma das senteças acima é verdadeira , vai o último else 
      } else {
        // Aqui definimos velocidade 0 para todos os motores
        motorDireitaMestre.set(0);
        motorEsquerdaMestre.set(0);
        rampa.set(0);
      } */
  } 

    // Esse é o autonomus que será utilizado caso o robô comece na esquerda
    private void Esquerda(double time) {
      // Aqui fazemos uma média dos valores recebidos pelos sensores, cada um por segundo, 
      // já que ele retorna vários valores e não dar problema na hora
      double currentValue1 = sharpSensor1.getValue();
      double currentValue2 = sharpSensor2.getValue();
      sensorValue1[index] = currentValue1;
      sensorValue2[index] = currentValue2;
      index = (index + 1) % NUM_VALUES;
      double sum = 0.0;
    for (double value : sensorValue1) {
        sum += value;
    }
    double average1 = sum / NUM_VALUES;

    for (double value : sensorValue2) {
      sum += value;
  }
  double average2 = sum / NUM_VALUES;

    // Envie a média para o SmartDashboard
    SmartDashboard.putNumber("Sharp Sensor Average1", average1);
    SmartDashboard.putNumber("Sharp Sensor Average2", average2);
      // Definir o tempo inicial e limite dessa parte dos if
      if(time - startTime < 1.7) {
        // Aqui é definido a velocidade do robô para ele ir para frente
        motorDireitaMestre.set(0.3385);
        motorEsquerdaMestre.set(0.30);
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 3 && time - startTime < 3.5) {
        // Aqui é definido a velocidade do robô para ele girar no próprio eixo, para a esquerda
        motorDireitaMestre.set(-0.3385);
        motorEsquerdaMestre.set(0.30);
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 3.5 && time - startTime < 5) {
        // Aqui é definido a velocidade do robô para ele ir para frente
        motorDireitaMestre.set(0.3385);
        motorEsquerdaMestre.set(0.30);
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 5.5 && time - startTime < 7.5) {
        // Aqui definimos a distância que o robô deve está do recife, para pode soltar o coral no recife
        if(average1 > 500 && average1 < 1600 && average2 > 500 && average2 < 1600) {
          // Aqui definimos a velocidade do motor da rampa, para soltar o coral
           rampa.set(-0.37);
               }
          else {
          // Definir a velocidade do motor para zero
           rampa.set(0);
          }
        // Quando nenhuma das senteças acima é verdadeira, vai o último else 
      } else {
        // Aqui definimos velocidade 0 para todos os motores
        motorDireitaMestre.set(0);
        motorEsquerdaMestre.set(0);
        rampa.set(0);
      }
    }
    

// Esse é o autonomus que será utilizado caso o robô comece no meio
    private void Meio(double time) {
      // Aqui fazemos uma média dos valores recebidos pelos sensores, cada um por segundo, 
      // já que ele retorna vários valores e não dar problema na hora
      double currentValue1 = sharpSensor1.getValue();
      double currentValue2 = sharpSensor2.getValue();
      sensorValue1[index] = currentValue1;
      sensorValue2[index] = currentValue2;
      index = (index + 1) % NUM_VALUES;
      
      // Média de todos os valores
      double sum = 0.0;
    for (double value : sensorValue1) {
        sum += value;
    }
    double average1 = sum / NUM_VALUES;

    for (double value : sensorValue2) {
      sum += value;
  }
  double average2 = sum / NUM_VALUES;

    // Envie a média para o SmartDashboard
    SmartDashboard.putNumber("Sharp Sensor Average1", average1);
    SmartDashboard.putNumber("Sharp Sensor Average2", average2);
      
      // Definir o tempo inicial e limite dessa parte dos if
      if(time - startTime < 3) {
        // Aqui é definido a velocidade do robô para ele ir para frente
        motorDireitaMestre.set(0.3385); /* 0.3385 */
        motorEsquerdaMestre.set(0.3); /* 0.3 */
        // Definir o tempo inicial e limite dessa parte dos if
      } else if (time - startTime >= 3 && time - startTime <= 7) {
        // Aqui definimos a distância que o robô deve está do recife, para pode soltar o coral no recife
         if(average1 >= 500 && average2 >= 500) {
        // Aqui definimos a velocidade do motor da rampa, para soltar o coral
          rampa.set(-0.30);
          
        // Aqui é definido a velocidade do robô para ele ir para frente
          motorDireitaMestre.set(0.2585); 
          motorEsquerdaMestre.set(0.25); 
        }
      
    
        else {
        // Definir a velocidade dos motores para zero
        rampa.set(0);
        motorDireitaMestre.set(0); 
        motorEsquerdaMestre.set(0); 
        }
        // Quando nenhuma das senteças acima é verdadeira, vai o último else 
      } else {
        // Aqui definimos velocidade 0 para todos os motores
        motorDireitaMestre.set(0);
        motorEsquerdaMestre.set(0);
        rampa.set(0);
      }
      
    }
    
  
    /** This function is called once when teleop is enabled. */
    @Override
    public void teleopInit() {
     
    }
    
      
        /** This function is called periodically during operator control. */
    @Override
    public void teleopPeriodic() {
    

      // Declaramos os botões do controle
      boolean y = controle.getYButton();
      boolean x = controle.getXButton();
      boolean a = controle.getAButton();
      boolean b = controle.getBButton();
      boolean lb = controle.getRawButton(5);
      double rt = controle.getRawAxis(3);
      double lt = controle.getRawAxis(2);
      boolean rb = controle.getRawButton (6);
      boolean quickTurn = controle.getRawButton(10);
      // Definição do valor que o rt manda para double, para conseguimos fazer o acelerador da garra
      double multiplicador = rt;
 
      

      
    
/* 
      if (lb) {
           if (currentPosition <= targetPosition) {
                garra.set(0.2);
              } 
          else {
              garra.set(0);
              }
              } 
          else if (rt >= 0.1 && rt <= 1) {
                if (currentPosition >= initialPosition) {
                garra.set(-0.7*multiplicador); 
              } else {
                garra.set(0);
              }
            } else {
              garra.set(0);
            } */
      
      // Aqui definimos o movimento do motor da garra
      // Se lb for apertado a garra vai descer
      if(lb) {
        // Definindo a velocidade da descida da garra
        garra.set(0.2);
        // Se rt for apertado a garra vai subir e dependendo do quando tiver pressionado 
        // a velocidade vai variar
      } else if (rt >= 0.1 && rt <= 1) {
        // Definindo a velocidade da subida da garra
        garra.set(-0.3*multiplicador); 
        // Se nenhum dos dois botões acima for pressionado, então a garra não se move
      } else {
        // Definindo para 0 a velocidade da garra
        garra.set(0);
      }
     
    // Aqui definimos a movimentação do motor da rampa
    // Se o botão y for apertado, solta o coral dentro da rampa
    if(y){
      // Definindo a velocidade da rampa
      rampa.set(-0.37);
    // Se o botão rb for apertado, vai rodar o motor da rampa para o lado de dentro para que o coral seja ajeitado e caso ele trave
    } else if(rb) {
      // Definindo a velocidade da rampa
      rampa.set(0.37);
    }
    // Se não tiver apertado, então o motor da rampa não gira
    else{
      // Definindo a velocidade da rampa para 0
      rampa.set(0);
    }
      // Aqui definimos a movimentação do motor do agarrador
      // Se o botão a for pressionado, agarrará a alga
    if(a){
      // Definindo a velocidade do agarrador
      agarrador.set(0.2);
      // Se o botão b for pressionado, soltará a alga
    }else if(b) {
      // Definindo a velocidade do agarrador
      agarrador.set(-0.3);
    }
    // Caso nenhum dos botões acima seja pressionado, então o motor do agarrador não movimentará
    else{
      // Definindo a velocidade do agarrador para 0
      agarrador.set(0);
    }
     
    double speed = -controle.getLeftY(); /* Define funcionalidade do botão analógico esquerdo */
    double turn = -controle.getRightX(); /* Define funcionalidade do botão analógico direito */

    // Define o limite de velocidade e aplica acelerador caso o botão lt seja pressionado
    if (lt >= 0.1 && lt <= 1) {
      speed = speed*0.8;
      turn = turn*0.8;
    } else {
      speed = speed*0.5;
      turn = turn*0.5;
    }
    

    
    // Definição da velocidade e trajeto pelos analogicos
    chassi.curvatureDrive(speed, turn, quickTurn); 
    //chassi.tankDrive(speed, turn);
    
    }
  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    // quando o robô estiver em modo disabled, nada dele se movimentará
    motorDireitaMestre.set(0);
    motorEsquerdaMestre.set(0);
    garra.set(0);
    rampa.set(0);
    agarrador.set(0);
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
