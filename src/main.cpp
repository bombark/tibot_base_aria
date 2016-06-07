#include <stdio.h>
#include <iostream>
#include <tiobj.hpp>
#include <tisys.hpp>
#include <Aria/Aria.h>


using namespace std;


int   G_id;
FILE* G_SONAR_FD;
FILE* G_pose_fd;

int G_count = 0;
int G_speed = 0;
int G_rotation = 0;

void readSonars(ArRobot& robot, int numSonar){
	char angle[64], value[64];
	G_id += 1;
	ArSensorReading* sonarReading;
	string res;
	for (int i=0; i < numSonar; i++){
		sonarReading = robot.getSonarReading(i);
		sprintf(value,"v%d=%05d;", i, sonarReading->getRange());
		res += value;
		//cout << "Sonar reading " << i << " = " << sonarReading->getRange() << " Angle " << i << " = " << sonarReading->getSensorTh() << "\n";
	}
	res += "\n";
	fseek(G_SONAR_FD, SEEK_SET, 0);
	fwrite(res.c_str(), sizeof(char), res.size(), G_SONAR_FD);
}


void readPosition(ArRobot& robot){
	ArPose pose = robot.getPose();
	fseek(G_pose_fd, SEEK_SET, 0);
	fprintf(G_pose_fd, "x=%0.6f;y=%0.6f;th=%0.6f;\n", pose.getX(), pose.getY(), pose.getTh());
}

void readMotors(){
	TiObj motor;
	motor.set("speed",G_speed);
	motor.set("rotation",G_rotation);
	motor.saveFile("motors");
}


void setMotors(ArRobot& robot){
	TiObj motor;
	motor.loadFile("cmd");

	if ( motor.has("speed") || motor.has("speed_i") || motor.has("rotation") || motor.has("rotation_i") ){
		G_count = 1;
		cout << "[LOG]: " << motor;
		robot.lock();
		if ( motor.has("speed") )
			G_speed = motor.atInt("speed");
			robot.setVel( G_speed );
		if ( motor.has("rotation") ) 
			G_rotation = motor.atInt("rotation");
			robot.setRotVel( G_rotation );
		if ( motor.has("speed_i") ){
			if ( abs(G_speed + motor.atInt("speed_i")) < 200){
				G_speed += motor.atInt("speed_i");
				robot.setVel( G_speed );
			}
		}
		if ( motor.has("rotation_i") ){
			if ( abs(G_rotation + motor.atInt("rotation_i")) < 60){
				G_rotation += motor.atInt("rotation_i");
				robot.setRotVel( G_rotation );
			}
		}
		robot.unlock();

		FILE* fd = fopen("cmd","w");
		fclose(fd);
	}
	//
}


void fake(){
	//fprintf(G_pose_fd, "x=%0.6f;y=%0.6f;th=%0.6f;\n", pose.getX(), pose.getY(), pose.getTh());


}




int main(int argc, char **argv){

	TiObj params;
	params.loadText(getenv("params"));
	string url = params.atStr("url");
	chdir(url.c_str());

	G_id = 0;
	G_SONAR_FD = fopen("sonars","w");
	G_pose_fd  = fopen("pose","w");

	FILE* aux = fopen("cmd","w");
	fclose(aux);


	char* robot_argv[] = {argv[0],"-robotPort","/dev/ttyUSB0",0};
	int   robot_argc = 3;


	Aria::init();
	ArRobot robot;
	ArArgumentParser parser(&robot_argc, robot_argv);
	ArSimpleConnector connector(&parser);


	parser.loadDefaultArguments();


	Aria::logOptions();



	if (!connector.parseArgs()){
		cout << "Unknown settings\n";
		Aria::exit(0);
		exit(1);
	}



	if (!connector.connectRobot(&robot)){
		cout << "Unable to connect\n";
		Aria::exit(0);
		exit(1);
	}



	robot.runAsync(true);
	robot.lock();
	robot.comInt(ArCommands::ENABLE, 1);
	robot.unlock();



	//ArSonarDevice sonar;
	//robot.addRangeDevice(&sonar);


	int numSonar = robot.getNumSonar();
	while(1){
		readPosition(robot);
		readMotors();
		readSonars(robot, 8);
		setMotors(robot);

		if ( (G_count%50) == 0 ){


			robot.lock();
			if ( abs(G_rotation) >= 50 ){
				if ( G_rotation > 0 )
					G_rotation -= 50;
				else
					G_rotation += 50;
			} else
				G_rotation = 0;

			if ( abs(G_speed) >= 20 ){
				if ( G_speed > 0 )
					G_speed -= 20;
				else
					G_speed += 20;
			} else
				G_speed = 0;

			robot.setVel( G_speed );
			robot.setRotVel( G_rotation );
			robot.unlock();
		}

		G_count += 1;
		usleep(20000);
	}

	fclose(G_SONAR_FD);
	fclose(G_pose_fd);
	Aria::exit(0);
}
