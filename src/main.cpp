#include <stdio.h>
#include <iostream>
#include <tiobj.hpp>
#include <tisys.hpp>
#include <Aria/Aria.h>
#include <unistd.h>
#include <sys/file.h>
#include <cassert>

using namespace std;


int   G_id;
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
	}
	res += "\n";

	FILE* fd = fopen("sonars","w");
	flock( fileno(fd), LOCK_SH );
	fwrite(res.c_str(), sizeof(char), res.size(), fd);
	flock( fileno(fd), LOCK_UN );
	fclose(fd);
}


void readPosition(ArRobot& robot){
	ArPose pose = robot.getPose();
	FILE* fd = fopen("odom","w");
	flock( fileno(fd), LOCK_SH );
	fprintf(fd, "x=%0.6f;y=%0.6f;th=%0.6f;\n", pose.getX(), pose.getY(), pose.getTh());
	flock( fileno(fd), LOCK_UN );
	fclose(fd);
}

void readMotors(){
	TiObj motor;
	motor.set("speed",G_speed);
	motor.set("rotation",G_rotation);
	motor.save("motors", true);
}


void setMotors(ArRobot& robot){
	TiObj motor(true, "cmd");
	truncate("cmd", 0);

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
	}
	//
}


void sig_hnd(int sig){
	Aria::exit(0);
	printf("Aria Exit");
}


int main(int argc, char **argv){

	TiObj params;
	params.loadText(getenv("params"));
	string url = params.atStr("url");
	assert(  chdir(url.c_str())  );

	G_id = 0;
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

	signal( SIGINT, sig_hnd );

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

}
