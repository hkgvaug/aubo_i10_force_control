# AUBO SDK 接口索引

由 `include/aubo_driver/serviceinterface.h` 自动生成。

- 头文件：`aubo_driver/include/aubo_driver/serviceinterface.h`（接口）
- 类型定义：`aubo_driver/include/aubo_driver/AuboRobotMetaType.h`
- 库：`aubo_driver/lib/lib64/aubocontroller/libauborobotcontroller.{so,a}`
- 类名：`ServiceInterface`（**注意：不在 `aubo_robot_namespace` 里**）

共 170 个公开方法。

## 本项目用到的（按用途分组）

### 透传/实时下发 (Tcp2Canbus)

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceEnterTcp2CanbusMode` | `int robotServiceEnterTcp2CanbusMode();` | TCP 转 CAN 透传模式 |
| `robotServiceLeaveTcp2CanbusMode` | `int robotServiceLeaveTcp2CanbusMode();` |  |
| `robotServiceSetRobotPosData2Canbus` | `int robotServiceSetRobotPosData2Canbus(double jointAngle[aubo_robot_namespace::ARM_DOF]);` |  |
| `robotServiceSetRobotPosData2Canbus` | `int robotServiceSetRobotPosData2Canbus(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);` |  |

### 系统/连接

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceLogin` | `int robotServiceLogin(const char* host, int port, const char *userName, const char* possword);` | 该接口的成功是调用其他接口的前提。 |
| `robotServiceLogin` | `int robotServiceLogin(const char* host, int port, const char *userName, const char* possword, aubo_robot_namespace::RobotType &robotType, aubo_robot_n` |  |
| `robotServiceGetConnectStatus` | `void robotServiceGetConnectStatus(bool &connectStatus);` |  |
| `robotServiceLogout` | `int robotServiceLogout();` |  |
| `rootServiceRobotStartup` | `int rootServiceRobotStartup(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam, uint8 collisionClass, bool readPose, bool staticCollisio` |  |
| `robotServiceRobotShutdown` | `int robotServiceRobotShutdown(bool IsBolck = true);` |  |
| `robotServiceGetIsRealRobotExist` | `int robotServiceGetIsRealRobotExist(bool &value);` |  |

### 状态实时获取

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceSetRealTimeJointStatusPush` | `int robotServiceSetRealTimeJointStatusPush(bool enable);` |  |
| `robotServiceRegisterRealTimeJointStatusCallback` | `int robotServiceRegisterRealTimeJointStatusCallback(RealTimeJointStatusCallback ptr, void *arg);` | 注册回调函数后,服务器实时推送当前的关节状态信息. |
| `robotServiceGetRobotDevInfoService` | `int robotServiceGetRobotDevInfoService(aubo_robot_namespace::RobotDevInfo &devInfo);` |  |
| `robotServiceGetRobotCurrentState` | `int robotServiceGetRobotCurrentState(aubo_robot_namespace::RobotState &state);` |  |
| `robotServiceGetRobotDiagnosisInfo` | `int robotServiceGetRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis &robotDiagnosisInfo);` |  |
| `robotServiceGetCurrentWaypointInfo` | `int robotServiceGetCurrentWaypointInfo(aubo_robot_namespace::wayPoint_S &wayPoint);` |  |

### 运动 (MoveJ/MoveL/MoveP)

| 方法 | 签名 | 说明 |
|---|---|---|
| `initMoveRelativeDataType` | `static void initMoveRelativeDataType(aubo_robot_namespace::MoveRelative &moveRelative);` |  |
| `robotServiceInitGlobalMoveProfile` | `int robotServiceInitGlobalMoveProfile();` | 调用此函数时机械臂不运动，该函数初始化各个运动属性为默认值。 |
| `robotServiceSetGlobalMoveJointMaxAcc` | `int robotServiceSetGlobalMoveJointMaxAcc (const aubo_robot_namespace::JointVelcAccParam &moveMaxAcc);` | 设置和获取　关节型　运动的最大速度和加速度 |
| `robotServiceSetGlobalMoveJointMaxVelc` | `int robotServiceSetGlobalMoveJointMaxVelc(const aubo_robot_namespace::JointVelcAccParam &moveMaxVelc);` |  |
| `robotServiceSetMoveRelativeParam` | `int robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnBase); //基于基座标系` |  |
| `robotServiceSetMoveRelativeParam` | `int robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnUserCoord, const aubo_robot_namespace::CoordCalibrateByJo` |  |
| `robotServiceJointMove` | `int robotServiceJointMove(aubo_robot_namespace::wayPoint_S &wayPoint, bool IsBolck);` | 属于关节型运动,调用该函数机械臂开始运动 |
| `robotServiceJointMove` | `int robotServiceJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF], bool IsBolck);` |  |
| `robotServiceFollowModeJointMove` | `int robotServiceFollowModeJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF]);` |  |
| `robotMoveLineToTargetPositionByRelative` | `int robotMoveLineToTargetPositionByRelative(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::Move` |  |
| `robotMoveLineToTargetPosition` | `int robotMoveLineToTargetPosition(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::Pos &toolEndPo` |  |
| `robotMoveFastStop` | `int robotMoveFastStop();` |  |
| `robotServiceSetWeaveMoveParameters` | `int robotServiceSetWeaveMoveParameters(const aubo_robot_namespace::WeaveMove &weaveMove);` |  |

### IO

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(const std::vector<aubo_robot_namespace::RobotIoType> ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &statusVe` |  |
| `robotServiceSetBoardIOStatus` | `int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double value);` |  |
| `robotServiceSetBoardIOStatus` | `int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int addr, double value);` |  |
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double &value);` |  |
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int addr, double &value);` |  |

### 工具/坐标系/负载

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceSetToolDynamicsParam` | `int robotServiceSetToolDynamicsParam(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);` | 设置工具的动力学参数　 **/ |
| `robotServiceSetToolKinematicsParam` | `int robotServiceSetToolKinematicsParam(const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);` | 设置工具的运动学参数　 **/ |
| `robotServiceGetRobotCollisionCurrentService` | `int robotServiceGetRobotCollisionCurrentService(int &collisionGrade);` |  |
| `robotServiceCollisionRecover` | `int robotServiceCollisionRecover();` |  |

### 急停/安全

| 方法 | 签名 | 说明 |
|---|---|---|
| `robotServiceSetRobotMaxACC` | `int robotServiceSetRobotMaxACC(int maxAcc);` |  |
| `robotServiceGetMacCommunicationStatus` | `int robotServiceGetMacCommunicationStatus(bool &value);` |  |

### 运动学工具

| 方法 | 签名 | 说明 |
|---|---|---|
| `initWayPointDataType` | `static void initWayPointDataType(aubo_robot_namespace::wayPoint_S &wayPoint);` |  |
| `quaternionToRPY` | `int quaternionToRPY(const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy);` |  |
| `RPYToQuaternion` | `int RPYToQuaternion(const aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori);` |  |

## 全部接口

| 方法 | 签名 |
|---|---|
| `initPosDataType` | `static void initPosDataType(aubo_robot_namespace::Pos &postion);` |
| `initOriDataType` | `static void initOriDataType(aubo_robot_namespace::Ori &ori);` |
| `initMoveRelativeDataType` | `static void initMoveRelativeDataType(aubo_robot_namespace::MoveRelative &moveRelative);` |
| `initWayPointDataType` | `static void initWayPointDataType(aubo_robot_namespace::wayPoint_S &wayPoint);` |
| `initToolInEndDescDataType` | `static void initToolInEndDescDataType(aubo_robot_namespace::ToolInEndDesc &toolInEndDesc);` |
| `initCoordCalibrateByJointAngleAndToolDataType` | `static void initCoordCalibrateByJointAngleAndToolDataType(aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coord);` |
| `initToolInertiaDataType` | `static void initToolInertiaDataType(aubo_robot_namespace::ToolInertia &toolInertia);` |
| `initToolDynamicsParamDataType` | `static void initToolDynamicsParamDataType(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);` |
| `robotServiceLogin` | `int robotServiceLogin(const char* host, int port, const char *userName, const char* possword);` |
| `robotServiceLogin` | `int robotServiceLogin(const char* host, int port, const char *userName, const char* possword, aubo_robot_namespace::RobotType &robotType, aubo_robot_namespace::` |
| `robotServiceGetConnectStatus` | `void robotServiceGetConnectStatus(bool &connectStatus);` |
| `robotServiceLogout` | `int robotServiceLogout();` |
| `robotServiceRobotHandShake` | `int robotServiceRobotHandShake(bool isBlock);` |
| `robotServiceSetRealTimeJointStatusPush` | `int robotServiceSetRealTimeJointStatusPush(bool enable);` |
| `robotServiceRegisterRealTimeJointStatusCallback` | `int robotServiceRegisterRealTimeJointStatusCallback(RealTimeJointStatusCallback ptr, void *arg);` |
| `robotServiceSetRealTimeRoadPointPush` | `int robotServiceSetRealTimeRoadPointPush(bool enable);` |
| `robotServiceRegisterRealTimeRoadPointCallback` | `int robotServiceRegisterRealTimeRoadPointCallback(const RealTimeRoadPointCallback ptr, void *arg);` |
| `robotServiceSetRealTimeEndSpeedPush` | `int robotServiceSetRealTimeEndSpeedPush(bool enable);` |
| `robotServiceRegisterRealTimeEndSpeedCallback` | `int robotServiceRegisterRealTimeEndSpeedCallback(const RealTimeEndSpeedCallback ptr, void *arg);` |
| `robotServiceRegisterRobotEventInfoCallback` | `int robotServiceRegisterRobotEventInfoCallback(RobotEventCallback ptr, void *arg);` |
| `recvRealTimeJointStatusPushCallback` | `static void recvRealTimeJointStatusPushCallback(const aubo_robot_namespace::JointStatus *jointStatus, int size, void *arg);` |
| `recvRealTimeWaypointPushCallback` | `static void recvRealTimeWaypointPushCallback(const aubo_robot_namespace::wayPoint_S *waypoint,void *arg);` |
| `recvRealTimeEndSpeedPushCallback` | `static void recvRealTimeEndSpeedPushCallback(double speed, void *arg);` |
| `recvRobotEventPushCallback` | `static void recvRobotEventPushCallback(const aubo_robot_namespace::RobotEventInfo *info, void *arg);` |
| `robotServiceInitGlobalMoveProfile` | `int robotServiceInitGlobalMoveProfile();` |
| `robotServiceSetGlobalMoveJointMaxAcc` | `int robotServiceSetGlobalMoveJointMaxAcc (const aubo_robot_namespace::JointVelcAccParam &moveMaxAcc);` |
| `robotServiceSetGlobalMoveJointMaxVelc` | `int robotServiceSetGlobalMoveJointMaxVelc(const aubo_robot_namespace::JointVelcAccParam &moveMaxVelc);` |
| `robotServiceGetGlobalMoveJointMaxAcc` | `void robotServiceGetGlobalMoveJointMaxAcc (aubo_robot_namespace::JointVelcAccParam &moveMaxAcc);` |
| `robotServiceGetGlobalMoveJointMaxVelc` | `void robotServiceGetGlobalMoveJointMaxVelc(aubo_robot_namespace::JointVelcAccParam &moveMaxVelc);` |
| `robotServiceSetGlobalMoveEndMaxLineAcc` | `int robotServiceSetGlobalMoveEndMaxLineAcc (double moveMaxAcc);` |
| `robotServiceSetGlobalMoveEndMaxLineVelc` | `int robotServiceSetGlobalMoveEndMaxLineVelc(double moveMaxVelc);` |
| `robotServiceGetGlobalMoveEndMaxLineAcc` | `void robotServiceGetGlobalMoveEndMaxLineAcc (double &moveMaxAcc);` |
| `robotServiceGetGlobalMoveEndMaxLineVelc` | `void robotServiceGetGlobalMoveEndMaxLineVelc(double &moveMaxVelc);` |
| `robotServiceSetGlobalMoveEndMaxAngleAcc` | `int robotServiceSetGlobalMoveEndMaxAngleAcc (double moveMaxAcc);` |
| `robotServiceSetGlobalMoveEndMaxAngleVelc` | `int robotServiceSetGlobalMoveEndMaxAngleVelc(double moveMaxVelc);` |
| `robotServiceGetGlobalMoveEndMaxAngleAcc` | `void robotServiceGetGlobalMoveEndMaxAngleAcc (double &moveMaxAcc);` |
| `robotServiceGetGlobalMoveEndMaxAngleVelc` | `void robotServiceGetGlobalMoveEndMaxAngleVelc(double &moveMaxVelc);` |
| `robotServiceSetJerkAccRatio` | `int robotServiceSetJerkAccRatio(double acc);` |
| `robotServiceGetJerkAccRatio` | `void robotServiceGetJerkAccRatio(double &acc);` |
| `robotServiceClearGlobalWayPointVector` | `void robotServiceClearGlobalWayPointVector();` |
| `robotServiceAddGlobalWayPoint` | `int robotServiceAddGlobalWayPoint(const aubo_robot_namespace::wayPoint_S &wayPoint);` |
| `robotServiceAddGlobalWayPoint` | `int robotServiceAddGlobalWayPoint(const double jointAngle[aubo_robot_namespace::ARM_DOF]);` |
| `robotServiceGetGlobalWayPointVector` | `void robotServiceGetGlobalWayPointVector(std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);` |
| `robotServiceSetGlobalBlendRadius` | `int robotServiceSetGlobalBlendRadius(float value);` |
| `robotServiceSetTrackPlaybackCycle` | `int robotServiceSetTrackPlaybackCycle(double second);` |
| `robotServiceGetGlobalCircularLoopTimes` | `int robotServiceGetGlobalCircularLoopTimes();` |
| `robotServiceSetGlobalCircularLoopTimes` | `void robotServiceSetGlobalCircularLoopTimes(int times);` |
| `robotServiceSetMoveRelativeParam` | `int robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnBase); //基于基座标系` |
| `robotServiceSetMoveRelativeParam` | `int robotServiceSetMoveRelativeParam(const aubo_robot_namespace::MoveRelative &relativeMoveOnUserCoord, const aubo_robot_namespace::CoordCalibrateByJointAngleAn` |
| `robotServiceSetNoArrivalAhead` | `int robotServiceSetNoArrivalAhead();` |
| `robotServiceSetArrivalAheadDistanceMode` | `int robotServiceSetArrivalAheadDistanceMode(double distance /*米*/);` |
| `robotServiceSetArrivalAheadTimeMode` | `int robotServiceSetArrivalAheadTimeMode(double second /*秒*/);` |
| `robotServiceSetArrivalAheadBlendDistanceMode` | `int robotServiceSetArrivalAheadBlendDistanceMode(double distance /*米*/);` |
| `robotServiceSetTeachCoordinateSystem` | `int robotServiceSetTeachCoordinateSystem(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem);` |
| `robotServiceJointMove` | `int robotServiceJointMove(aubo_robot_namespace::wayPoint_S &wayPoint, bool IsBolck);` |
| `robotServiceJointMove` | `int robotServiceJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF], bool IsBolck);` |
| `robotServiceFollowModeJointMove` | `int robotServiceFollowModeJointMove(double jointAngle[aubo_robot_namespace::ARM_DOF]);` |
| `robotServiceLineMove` | `int robotServiceLineMove(aubo_robot_namespace::wayPoint_S &wayPoint, bool IsBolck);` |
| `robotServiceLineMove` | `int robotServiceLineMove(double jointAngle[aubo_robot_namespace::ARM_DOF], bool IsBolck);` |
| `robotServiceRotateMove` | `int robotServiceRotateMove(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const double rotateAxisOnUserCoord[3], double rotateAngle, ` |
| `robotServiceTrackMove` | `int robotServiceTrackMove(aubo_robot_namespace::move_track subMoveMode, bool IsBolck);` |
| `robotMoveLineToTargetPositionByRelative` | `int robotMoveLineToTargetPositionByRelative(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::MoveRelative &` |
| `robotMoveJointToTargetPositionByRelative` | `int robotMoveJointToTargetPositionByRelative(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::MoveRelative ` |
| `getJointAngleByTargetPositionKeepCurrentOri` | `int getJointAngleByTargetPositionKeepCurrentOri(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem, const aubo_robot_namespace::Pos &too` |
| `robotMoveLineToTargetPosition` | `int robotMoveLineToTargetPosition(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::Pos &toolEndPositionOnUs` |
| `robotMoveJointToTargetPosition` | `int robotMoveJointToTargetPosition(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &userCoord, const aubo_robot_namespace::Pos &toolEndPositionOnU` |
| `robotServiceTeachStart` | `int robotServiceTeachStart(aubo_robot_namespace::teach_mode mode, bool direction);` |
| `robotServiceTeachStop` | `int robotServiceTeachStop();` |
| `rootServiceRobotMoveControl` | `int rootServiceRobotMoveControl(aubo_robot_namespace::RobotMoveControlCommand cmd);` |
| `robotMoveFastStop` | `int robotMoveFastStop();` |
| `robotServiceOfflineTrackWaypointAppend` | `int robotServiceOfflineTrackWaypointAppend(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);` |
| `robotServiceOfflineTrackWaypointAppend` | `int robotServiceOfflineTrackWaypointAppend(const char *fileName);` |
| `robotServiceOfflineTrackWaypointClear` | `int robotServiceOfflineTrackWaypointClear ();` |
| `robotServiceOfflineTrackMoveStartup` | `int robotServiceOfflineTrackMoveStartup (bool IsBolck);` |
| `robotServiceOfflineTrackMoveStop` | `int robotServiceOfflineTrackMoveStop ();` |
| `robotServiceEnterTcp2CanbusMode` | `int robotServiceEnterTcp2CanbusMode();` |
| `robotServiceLeaveTcp2CanbusMode` | `int robotServiceLeaveTcp2CanbusMode();` |
| `robotServiceSetRobotPosData2Canbus` | `int robotServiceSetRobotPosData2Canbus(double jointAngle[aubo_robot_namespace::ARM_DOF]);` |
| `robotServiceSetRobotPosData2Canbus` | `int robotServiceSetRobotPosData2Canbus(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointVector);` |
| `startupOfflineExcitTrajService` | `int startupOfflineExcitTrajService(const char *trackFile, aubo_robot_namespace::Robot_Dyn_identify_traj type, int subtype, bool isBolck);` |
| `getDynIdentifyResultsService` | `int getDynIdentifyResultsService(std::vector<int> &paramVector);` |
| `robotServiceRobotFk` | `int robotServiceRobotFk(const double *jointAngle, int size, aubo_robot_namespace::wayPoint_S &wayPoint);` |
| `robotServiceRobotIk` | `int robotServiceRobotIk(const double *startPointJointAngle,const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, aubo_robot_namespace` |
| `robotServiceRobotIk` | `int robotServiceRobotIk(const aubo_robot_namespace::Pos &position, const aubo_robot_namespace::Ori &ori, std::vector<aubo_robot_namespace::wayPoint_S> &wayPoint` |
| `robotServiceToolCalibration` | `int robotServiceToolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosVector, char poseCalibMethod, aubo_robot_namespace::ToolInEndDes` |
| `robotServiceToolCalibration` | `int robotServiceToolCalibration(const std::vector<aubo_robot_namespace::wayPoint_S> &wayPointPosCalibVector, const std::vector<aubo_robot_namespace::wayPoint_S>` |
| `robotServiceCheckUserCoordinate` | `int robotServiceCheckUserCoordinate(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem);` |
| `robotServiceUserCoordinateCalibration` | `int robotServiceUserCoordinateCalibration(const aubo_robot_namespace::CoordCalibrateByJointAngleAndTool &coordSystem, double bInWPos[3], double bInWOri[9], doub` |
| `robotServiceOriMatrixToQuaternion` | `int robotServiceOriMatrixToQuaternion(double eerot[], aubo_robot_namespace::Ori &result);` |
| `quaternionToRPY` | `int quaternionToRPY(const aubo_robot_namespace::Ori &ori, aubo_robot_namespace::Rpy &rpy);` |
| `RPYToQuaternion` | `int RPYToQuaternion(const aubo_robot_namespace::Rpy &rpy, aubo_robot_namespace::Ori &ori);` |
| `rootServiceRobotControl` | `int rootServiceRobotControl(const aubo_robot_namespace::RobotControlCommand cmd);` |
| `robotServicePowerControl` | `int robotServicePowerControl(bool value);` |
| `robotServiceReleaseBrake` | `int robotServiceReleaseBrake();` |
| `rootServiceRobotStartup` | `int rootServiceRobotStartup(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam, uint8 collisionClass, bool readPose, bool staticCollisionDetect, i` |
| `robotServiceRobotShutdown` | `int robotServiceRobotShutdown(bool IsBolck = true);` |
| `robotServiceSetNoneToolDynamicsParam` | `int robotServiceSetNoneToolDynamicsParam();` |
| `robotServiceSetToolDynamicsParam` | `int robotServiceSetToolDynamicsParam(const aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);` |
| `robotServiceGetToolDynamicsParam` | `int robotServiceGetToolDynamicsParam(aubo_robot_namespace::ToolDynamicsParam &toolDynamicsParam);` |
| `robotServiceSetNoneToolKinematicsParam` | `int robotServiceSetNoneToolKinematicsParam();` |
| `robotServiceSetToolKinematicsParam` | `int robotServiceSetToolKinematicsParam(const aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);` |
| `robotServiceGetToolKinematicsParam` | `int robotServiceGetToolKinematicsParam(aubo_robot_namespace::ToolKinematicsParam &toolKinematicsParam);` |
| `robotServiceGetRobotWorkMode` | `int robotServiceGetRobotWorkMode(aubo_robot_namespace::RobotWorkMode &mode);` |
| `robotServiceSetRobotWorkMode` | `int robotServiceSetRobotWorkMode(aubo_robot_namespace::RobotWorkMode mode);` |
| `robotServiceGetRobotGravityComponent` | `int robotServiceGetRobotGravityComponent(aubo_robot_namespace::RobotGravityComponent &gravityComponent);` |
| `robotServiceGetRobotCollisionCurrentService` | `int robotServiceGetRobotCollisionCurrentService(int &collisionGrade);` |
| `robotServiceSetRobotCollisionClass` | `int robotServiceSetRobotCollisionClass(int grade);` |
| `robotServiceGetRobotDevInfoService` | `int robotServiceGetRobotDevInfoService(aubo_robot_namespace::RobotDevInfo &devInfo);` |
| `robotServiceSetRobotMaxACC` | `int robotServiceSetRobotMaxACC(int maxAcc);` |
| `robotServiceCollisionRecover` | `int robotServiceCollisionRecover();` |
| `robotServiceGetRobotCurrentState` | `int robotServiceGetRobotCurrentState(aubo_robot_namespace::RobotState &state);` |
| `robotServiceGetMacCommunicationStatus` | `int robotServiceGetMacCommunicationStatus(bool &value);` |
| `robotServiceGetIsRealRobotExist` | `int robotServiceGetIsRealRobotExist(bool &value);` |
| `robotServiceGetJoint6Rotate360EnableFlag` | `int robotServiceGetJoint6Rotate360EnableFlag(bool &value);` |
| `robotServiceGetRobotJointStatus` | `int robotServiceGetRobotJointStatus(aubo_robot_namespace::JointStatus *jointStatus, int size);` |
| `robotServiceGetRobotDiagnosisInfo` | `int robotServiceGetRobotDiagnosisInfo(aubo_robot_namespace::RobotDiagnosis &robotDiagnosisInfo);` |
| `robotServiceGetJointAngleInfo` | `int robotServiceGetJointAngleInfo(aubo_robot_namespace::JointParam &jointParam);` |
| `robotServiceGetCurrentWaypointInfo` | `int robotServiceGetCurrentWaypointInfo(aubo_robot_namespace::wayPoint_S &wayPoint);` |
| `robotServiceSetRobotAtOriginPose` | `int robotServiceSetRobotAtOriginPose();` |
| `robotServiceSetRobotOrpePause` | `int robotServiceSetRobotOrpePause(uint8 data);` |
| `robotServiceSetRobotOrpeStop` | `int robotServiceSetRobotOrpeStop(uint8 data);` |
| `robotServiceSetRobotOrpeError` | `int robotServiceSetRobotOrpeError(uint8 data[], int len);` |
| `robotServiceClearSystemEmergencyStop` | `int robotServiceClearSystemEmergencyStop(uint8 data);` |
| `robotServiceClearReducedModeError` | `int robotServiceClearReducedModeError(uint8 data);` |
| `robotServiceRobotSafetyguardResetSucc` | `int robotServiceRobotSafetyguardResetSucc(uint8 data);` |
| `robotServiceGetBoardIOConfig` | `int robotServiceGetBoardIOConfig(const std::vector<aubo_robot_namespace::RobotIoType> &ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &configVector);` |
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(const std::vector<aubo_robot_namespace::RobotIoType> ioType, std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);` |
| `robotServiceSetBoardIOStatus` | `int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double value);` |
| `robotServiceSetBoardIOStatus` | `int robotServiceSetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int addr, double value);` |
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, std::string name, double &value);` |
| `robotServiceGetBoardIOStatus` | `int robotServiceGetBoardIOStatus(aubo_robot_namespace::RobotIoType type, int addr, double &value);` |
| `robotServiceIsOnlineMode` | `int robotServiceIsOnlineMode(bool &isOnlineMode);` |
| `robotServiceIsOnlineMasterMode` | `int robotServiceIsOnlineMasterMode(bool &isOnlineMasterMode);` |
| `robotServiceGetRobotSafetyConfig` | `int robotServiceGetRobotSafetyConfig(aubo_robot_namespace::RobotSafetyConfig &safetyConfig);` |
| `robotServiceSetRobotSafetyConfig` | `int robotServiceSetRobotSafetyConfig(const aubo_robot_namespace::RobotSafetyConfig &safetyConfig);` |
| `robotServiceGetOrpeSafetyStatus` | `int robotServiceGetOrpeSafetyStatus(aubo_robot_namespace::OrpeSafetyStatus &safetyStatus);` |
| `robotServiceSetToolPowerVoltageType` | `int robotServiceSetToolPowerVoltageType (aubo_robot_namespace::ToolPowerType type);` |
| `robotServiceGetToolPowerVoltageType` | `int robotServiceGetToolPowerVoltageType (aubo_robot_namespace::ToolPowerType &type);` |
| `robotServiceGetToolPowerVoltageStatus` | `int robotServiceGetToolPowerVoltageStatus(double &value);` |
| `robotServiceSetToolPowerTypeAndDigitalIOType` | `int robotServiceSetToolPowerTypeAndDigitalIOType(aubo_robot_namespace::ToolPowerType type, aubo_robot_namespace::ToolIOType io0, aubo_robot_namespace::ToolIOTyp` |
| `robotServiceSetToolDigitalIOType` | `int robotServiceSetToolDigitalIOType(aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::ToolIOType type);` |
| `robotServiceGetAllToolDigitalIOStatus` | `int robotServiceGetAllToolDigitalIOStatus(std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);` |
| `robotServiceSetToolDOStatus` | `int robotServiceSetToolDOStatus (aubo_robot_namespace::ToolDigitalIOAddr addr, aubo_robot_namespace::IO_STATUS value);` |
| `robotServiceSetToolDOStatus` | `int robotServiceSetToolDOStatus(std::string name, aubo_robot_namespace::IO_STATUS value);` |
| `robotServiceGetToolIoStatus` | `int robotServiceGetToolIoStatus(std::string name, double &value);` |
| `robotServiceGetAllToolAIStatus` | `int robotServiceGetAllToolAIStatus (std::vector<aubo_robot_namespace::RobotIoDesc> &statusVector);` |
| `robotServiceUpdateRobotBoardFirmware` | `int robotServiceUpdateRobotBoardFirmware(aubo_robot_namespace::update_board_firmware_cmd cmd, const void *data, uint16 length);` |
| `robotServiceGetBoardFirmwareUpdateResultService` | `int robotServiceGetBoardFirmwareUpdateResultService(bool &value);` |
| `robotServiceGetRobotEthernetDeviceName` | `int robotServiceGetRobotEthernetDeviceName(std::string &ethernetDeviceName);` |
| `robotServiceSetRobotJointOffset` | `int robotServiceSetRobotJointOffset(aubo_robot_namespace::RobotJointOffset &jointOffset);` |
| `robotServiceSetConveyorEncoderReset` | `int robotServiceSetConveyorEncoderReset(void);` |
| `robotServiceSetConveyorStartup` | `int robotServiceSetConveyorStartup(void);` |
| `robotServiceSetConveyorStop` | `int robotServiceSetConveyorStop(void);` |
| `robotServiceSetConveyorDir` | `int robotServiceSetConveyorDir(int dir);` |
| `robotServiceSetRobotCameraCalib` | `int robotServiceSetRobotCameraCalib(const aubo_robot_namespace::RobotCameraCalib &robotCameraCalib);` |
| `robotServiceSetConveyorVelc` | `int robotServiceSetConveyorVelc(const double conveyorVelc);` |
| `robotServiceSetEncoderValPerMeter` | `int robotServiceSetEncoderValPerMeter(const uint32_t &encoderValPerMeter);` |
| `robotServiceSetStartWindowUpstream` | `int robotServiceSetStartWindowUpstream(double startWindowUpstream);` |
| `robotServiceSetStartWindowDownstream` | `int robotServiceSetStartWindowDownstream(double startWindowDownstream);` |
| `robotServiceSetConveyorTrackDownstream` | `int robotServiceSetConveyorTrackDownstream(double trackDownstream);` |
| `robotServiceAppendObject2ConveyorTrackQueue` | `int robotServiceAppendObject2ConveyorTrackQueue(const aubo_robot_namespace::Pos &objectPos, const aubo_robot_namespace::Ori &objectOri, uint32_t timestamp);` |
| `robotServiceEnableConveyorTrack` | `int robotServiceEnableConveyorTrack();` |
| `robotServiceGetConveyorEncoderVal` | `int robotServiceGetConveyorEncoderVal(int &value);` |
| `robotServiceSetRobotConveyorTrackMaxVelc` | `int robotServiceSetRobotConveyorTrackMaxVelc(double robotConveyorTrackMaxVelc);` |
| `robotServiceSetRobotConveyorTrackMaxAcc` | `int robotServiceSetRobotConveyorTrackMaxAcc(double robotConveyorTrackMaxAcc);` |
| `robotServiceSetRobotConveyorSystemDelay` | `int robotServiceSetRobotConveyorSystemDelay(double robotConveyorSystemDelay);` |
| `robotServiceSetRobotTool` | `int robotServiceSetRobotTool(const aubo_robot_namespace::ToolInEndDesc &robotTool);` |
| `robotServiceSetWeaveMoveParameters` | `int robotServiceSetWeaveMoveParameters(const aubo_robot_namespace::WeaveMove &weaveMove);` |
| `robotServiceSetRobotRecognitionParam` | `int robotServiceSetRobotRecognitionParam(const aubo_robot_namespace::RobotRecongnitionParam &param);` |
| `robotServiceGetRobotRecognitionParam` | `int robotServiceGetRobotRecognitionParam(int type, aubo_robot_namespace::RobotRecongnitionParam &param);` |
