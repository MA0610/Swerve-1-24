package frc.robot;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
//import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
//import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
//import com.revrobotics.CANSparkMax.ControlType;
//import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.math.Conversions;
import frc.lib.util.SwerveModuleConstants;
import frc.robot.utils.RevUtils;
import frc.robot.utils.RevUtils.PID_SLOT;

public class SwerveModule {
    public int moduleNumber;
    private Rotation2d angleOffset;

    //private TalonFX mAngleMotor;
    //private TalonFX mDriveMotor;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    private SparkPIDController angController;
    private SparkPIDController driveController;
    private RelativeEncoder m_driveEncoder;
    private RelativeEncoder m_angEncoder;

    private CANcoder angleEncoder;

    private final SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    /* drive motor control requests */
    private final DutyCycleOut driveDutyCycle = new DutyCycleOut(0);
    private final VelocityVoltage driveVelocity = new VelocityVoltage(0);

    /* angle motor control requests */
    private final PositionVoltage anglePosition = new PositionVoltage(0);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANcoder(moduleConstants.cancoderID);
        angleEncoder.getConfigurator().apply(Robot.ctreConfigs.swerveCANcoderConfig);

        /* Angle Motor Config */
        //mAngleMotor = new TalonFX(moduleConstants.angleMotorID);
        //mAngleMotor.getConfigurator().apply(Robot.ctreConfigs.swerveAngleFXConfig);
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
        RevUtils.setTurnMotorConfig(mAngleMotor);

        resetToAbsolute();

        /* Drive Motor Config */
        //mDriveMotor = new TalonFX(moduleConstants.driveMotorID);
        //mDriveMotor.getConfigurator().apply(Robot.ctreConfigs.swerveDriveFXConfig);
        //mDriveMotor.getConfigurator().setPosition(0.0);
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
        RevUtils.setDriveMotorConfig(mDriveMotor);
        
        angController = mAngleMotor.getPIDController();
        driveController = mDriveMotor.getPIDController();
        m_angEncoder = mAngleMotor.getEncoder();
        m_driveEncoder = mDriveMotor.getEncoder();
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
      /*   desiredState = SwerveModuleState.optimize(desiredState, getState().angle); 
        mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations())); HERE
        setSpeed(desiredState, isOpenLoop);
    */
    desiredState = RevUtils.optimize(desiredState, getState().angle);
    //mAngleMotor.setControl(anglePosition.withPosition(desiredState.angle.getRotations()));
    angController.setReference(desiredState.angle.getRotations(), ControlType.kSmartMotion);
    
    setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
     /*    if(isOpenLoop){
            driveDutyCycle.Output = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.setControl(driveDutyCycle);
        }
        else {
            driveVelocity.Velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference);
            driveVelocity.FeedForward = driveFeedForward.calculate(desiredState.speedMetersPerSecond);
            mDriveMotor.setControl(driveVelocity);
        }
*/
if (isOpenLoop) {
    double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
    mDriveMotor.set(percentOutput);
  } else {
    PID_SLOT DRIVE_PID_SLOT = RobotBase.isReal() ? PID_SLOT.VEL_SLOT : PID_SLOT.SIM_SLOT;
    driveController.setReference(
            desiredState.speedMetersPerSecond,
            CANSparkMax.ControlType.kVelocity,
            DRIVE_PID_SLOT.ordinal()
    );
    }}

    public Rotation2d getCANcoder(){
        return Rotation2d.fromRotations(angleEncoder.getAbsolutePosition().getValue());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
        //mAngleMotor.setPosition(absolutePosition);
        
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.RPSToMPS(m_driveEncoder.getVelocity()/60, Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(m_angEncoder.getPosition())
            );
    
        }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.rotationsToMeters(m_driveEncoder.getPosition(), Constants.Swerve.wheelCircumference), 
            Rotation2d.fromRotations(m_angEncoder.getPosition())
        );
    }
}