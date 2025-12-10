#ifndef GHOSTMANAGER_H
#define	GHOSTMANAGER_H


#define GHOST_DATA 0x0062
#define MAX_LINEAR_SPEED 1
#define MAX_LINEAR_ACCEL 0.2 
#define MAX_ANGULAR_SPEED 2 * PI 
#define MAX_ANGULAR_ACCEL 2 * PI 
#define ANGLE_TOLERANCE 0.05 
#define DISTANCE_TOLERANCE 0.1 

typedef enum {
    IDLE,
    ROTATING,
    ADVANCING,
    LASTROTATE
} TrajectoryState;

typedef struct {
    TrajectoryState state;
    double x;
    double y;
    double theta;
    double linearSpeed;
    double angularSpeed;
    double targetX;
    double targetY;
    double angleToTarget;
    double distanceToTarget;   
} GhostPosition;


extern volatile GhostPosition ghostposition;

void UpdateTrajectory();
void SendGhostData();
void InitTrajectoryGenerator(void);
void rotationTarget(double currentTime);

#endif	/* GHOSTMANAGER_H */

