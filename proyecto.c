#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <pthread.h>
#include <semaphore.h>
#include <fcntl.h> 
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <sys/mman.h>
#include <sys/stat.h>

#define BUFFER_SIZE 100
#define PROC_END 2
#define PROC_RST 3
#define SHMSZ	27
#define MAX_CONFIG_ARGS 3
#define TRUE 1
#define FALSE 0

//TODO
//> A MECHANISM TO CONTROL THE NUMBER OF SAMPLES OF EACH SENSOR
//> A MECHANISM TO STORE THE PROCESSED VALUE OF EACH PAIR OF LASER AND GSCOPE SENSOR READINGS
//> A MECHANISM TO RESET THE INTERVAL TIME
//> A MECHANISM TO CALCULATE THE MEAN OF EACH SAMPLES BUFFER

/* vars for processes control */
int processes_count=0;
pid_t *sensor_processes;

/* shared process mem segments*/
char ** sensorData;
int *sensorID;
int shmem_ids[6];

/* vars for sharedMem keys control */
int keys_count=0;

/* vars for sensors Data management and computations */
char (*distanceBuffer)[BUFFER_SIZE][SHMSZ];
char (*gscopeBuffer)[BUFFER_SIZE][SHMSZ];
int *buffd_in;
int *buffg_in;
int *buffd_out;
int *buffg_out;
char oldDataValue[SHMSZ];

/* semaphores for sync */
sem_t **reader_mutexes;
sem_t *proc_sem, *reader_sem;
sem_t *buffer_mutexes;

/* File to load configs of sensors */
FILE *configfp;

/*File where changes to I,Q,W,T will be applied*/
FILE *changesfp;

/* Files to write results*/
FILE *logfp;
FILE *reslogfp;

/* vars used for analytics */
time_t actual_time;
time_t elapsed_time;
int op_per_sec=0;
int round_counter=1;

/* vars used for reading obstacles */
int I = 10;
int Q = 5;
float T = 0.9;
float W = 3.0;
int Q_backup= 5;


/* vars used for sensors sampling */
time_t base_time;
time_t current_time;
time_t delta_time;
unsigned int readyForSample;
unsigned int samplesTaken= FALSE;
unsigned int samplesBufferFull= FALSE;
int * samplesCounterBuffer;
float ** samplesBuffer;
float * means;

/* vars used for configs change control */
unsigned int onConfigChange= FALSE;
struct stat fileStat;



//prototypes
int recreate_process(pid_t *oldPid,int sensorKey);
int sensor_read(key_t shmkey, int proc_index);
void sigchild_handler(int sig);
void siguser_handler(int sig);
void *main_reader(void *param);
void *main_calculator(void *param);
void *changes_monitor(void *param);


/*=====Main Program (Main Process)=====*/
int main(int argc, const char * argv[])
{
	//vars for process creation
	int i;
  	int shm_status;
  	pid_t pid;
  	pid_t *pidp;
	char sem_name[10];
  
	//vars for threads creation
	key_t shmemKey;
	pthread_t *reading_threads;
	pthread_t *calculator_threads;
	pthread_t cmonitor_thread;
	pthread_attr_t attr;

	//vars for fetching and parsing configs
	char * config_string;
	char * token;
	const char sep[2]= ",";
	const char id_sep[2]= "|";
	const char *config_tokens[3];

	//vars for samples buffer checking
	unsigned int isVehicle= FALSE;
	unsigned int isObstacle= FALSE;
	float acum=0.0;
	float lasersMean;
	float lstdev=0;
	int j;


	//registering the handler for SIGCHILD signal
  signal(SIGCHLD,sigchild_handler);
  
	//Validating parameters on program execution
  if (argc!=2){
    fprintf(stderr,"Incorrect args!\n");
    printf("\nUsage: proyecto <config_file_path>\n");
    return 1;
  }

	//Opening the file that contains the configs
	configfp= fopen(argv[1],"r");

	if (configfp == NULL){
		fprintf(stderr,"Error opening config file: file is corrupted or does not exist!\n");
		return 1;
	}

	//Seeking the pointer of file to read entire content once
	fseek(configfp,0,SEEK_END);
	long size= ftell(configfp);
	rewind(configfp);
	config_string= malloc(size + 1);
	fread(config_string,size,1,configfp);
	fclose(configfp);

	config_string[size]= '\0';
	printf("Content of config file: %s\n",config_string);

	//Tokenizing config contents
	token= strtok(config_string,sep);
	for(i=0; (i < MAX_CONFIG_ARGS) && (token != NULL) ; i++){
			printf("%s\n",token);
			if(i>0 && atoi(token) == 0){
				fprintf(stderr,"ERROR PARSING CONFIG VALUES! Value %s must be an integer number greater than zero\n",token);	
				return 1;
			}
			config_tokens[i]= token;
			token= strtok(NULL,sep);
	}

	if(i != MAX_CONFIG_ARGS){
		fprintf(stderr,"MISS CONFIG ERROR! Insuficient parameters found in config file! (found %d, required %d)\n",i,MAX_CONFIG_ARGS);
		return 1;
	}

	//Initial reading process values
	I= atoi(config_tokens[1]);
	if(I <=0){
		fprintf(stderr,"MISS CONFIG ERROR! 'I' cannot be less or equal than zero\n");
		return 1;
	}

	Q= atoi(config_tokens[2]);
	if(Q <=0){
		fprintf(stderr,"MISS CONFIG ERROR! 'Q' cannot be less or equal than zero\n");
		return 1;
	}
	Q_backup= Q;
	
	//tokenizing shared memory ids
	token=strtok(config_tokens[0],id_sep);
	for(i=0; token != NULL; i++){
		printf("%s\n",token);
		shmem_ids[i]= atoi(token);
		if(shmem_ids[i] == 0){
			fprintf(stderr,"ERROR PARSING SHARED MEM IDS! Value %s must be an integer number\n",token);
			return 1;
		}
		token= strtok(NULL,id_sep);
	}

	if(i % 2 != 0){
		fprintf(stderr,"MISS CONFIG ERROR! You must provide each ID of laser and gscope, not only 1 of them (%d found) \n",i);
		return 1;
	}

	//opening files for logging readings and results
	logfp= fopen("readings.log","w+");
	reslogfp= fopen("results.log","w+");
  
	//Init arrays and allocating space for processes fork and mutex
  processes_count= i;
  sensor_processes= (pid_t *) malloc(processes_count * sizeof(pid_t *));
	reader_mutexes= (sem_t **) malloc(processes_count * sizeof(sem_t **));
	buffer_mutexes= malloc(sizeof(sem_t) * (int) processes_count / 2);
	sensorData= (char **) malloc(sizeof(char **) * processes_count);
	sensorID= (int *) mmap(NULL, sizeof(int *) * processes_count, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0);
	
	//Allocating space for shared memory blocks and creating mutexes
	for(i = 0; i < processes_count; i++)
	{
		sprintf(sem_name,"/rSem%d",i);
		sensorData[i]= (char *) mmap(NULL, sizeof(char *) * SHMSZ, PROT_READ|PROT_WRITE, MAP_SHARED|MAP_ANONYMOUS, -1, 0);
		reader_mutexes[i]= sem_open(sem_name, O_CREAT | O_EXCL, 0633, 1);
		sem_unlink(sem_name);
	}

	//Allocating space for reading buffers and index reading buffers;
	gscopeBuffer= malloc(sizeof(char [BUFFER_SIZE][SHMSZ]) * (int) processes_count / 2);
	distanceBuffer= malloc(sizeof(char [BUFFER_SIZE][SHMSZ]) * (int) processes_count / 2);
	buffd_in= (int *) calloc((int) processes_count / 2,sizeof(int));
	buffg_in= (int *) calloc((int) processes_count / 2,sizeof(int) );
	buffd_out= (int *) calloc((int) processes_count / 2,sizeof(int));
	buffg_out= (int *) calloc((int) processes_count / 2,sizeof(int));
	
	//Allocating samples and counters Buffer 
	samplesBuffer= (float **) malloc(sizeof(float *) * (int) processes_count / 2);
	for(i=0; i < (int) processes_count / 2; i++){
			samplesBuffer[i]= (float *) calloc(Q,sizeof(float));
	}
	samplesCounterBuffer= (int *) calloc(processes_count,sizeof(int));
	means= calloc((int) processes_count / 2, sizeof(float));

	//Buffer mutex allocation and creation
	for(i= 0; i < (int) processes_count / 2 ; i++){
		sem_init(&buffer_mutexes[i],1,1);
	}

	//Creating the processes that collect sensors values
	pidp= sensor_processes;
	for(i=0; i<processes_count; i++){
    pid= fork();
    if(pid<0){
      fprintf(stderr,"\n**An error has ocurred during fork()!**\n");
      return 1;                     
    }

    if(pid==0)
    {
      printf("\nChild --> pid is %d\n", getpid());
      signal(SIGUSR1,siguser_handler);
      signal(SIGUSR2,siguser_handler);
			shmemKey= shmem_ids[i];
			shm_status= sensor_read(shmemKey,i);			
      return shm_status;
    }
     
    if(pid>0){
			keys_count++;
      *pidp= pid;
			++pidp;
    } 
 	}
	printf("\nParent --> pid is %d\n", getpid());

	//Allocating and creating threads for reading and proccesing data
	reading_threads= (pthread_t *) malloc(sizeof(pthread_t *) * processes_count);
 	calculator_threads= (pthread_t *) malloc(sizeof(pthread_t *) * processes_count);
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
	for(i=0; i< processes_count; i++){
		pthread_create(&reading_threads[i],&attr,main_reader,(void *) i);
	}
	for(i=0; i< (int) processes_count/2; i++){
		pthread_create(&calculator_threads[i],&attr,main_calculator,(void *) i);
	}
	pthread_create(&cmonitor_thread,&attr,changes_monitor,NULL);
	
	//actual_time= time(NULL);

	base_time= time(NULL);

	/*On this part, the system will be monitoring the intervals,
	  monitoring counting buffers, samples Buffer and it will
	  make the computations to check if the object is a vehicle
	  or an obstracle
	*/
	while(1){
  		sleep(1);

		/*Checking if the difference of actual time and base time
		  is equal or greater than sampling interval
		*/
		current_time= time(NULL);
		delta_time= current_time - base_time;
		if(delta_time >= I && readyForSample == FALSE){
			fprintf(stderr,"TAKING %d SAMPLES!\n ",Q);
			readyForSample= TRUE;
			base_time= time(NULL);
		}

		//Checking if all the samples of all sensors have been taken
		samplesTaken= TRUE;
		for(i=0; i < processes_count && readyForSample == TRUE; i++){
			if(samplesCounterBuffer[i] != Q_backup){
				fprintf(stderr,"Counter in index %d is not ready yet: %d\n ",i,samplesCounterBuffer[i]);
				samplesTaken= FALSE;
			}
		}

		/*If all the samples were taken, proceed to
		free the counter buffer and restart the interval*/
		if(samplesTaken && readyForSample){
			for(i=0; i < processes_count; i++){
				samplesCounterBuffer[i] = 0;
			}
			readyForSample= FALSE;
			fprintf(stderr,"SAMPLES TAKEN, RESTARTING INTERVAL\n ");
			base_time= time(NULL);
			samplesTaken=FALSE;
		}

		/*Checking if all the samples were processed and stored on
		samples buffer*/
		samplesBufferFull=TRUE;
		for(i=0;i < (int)processes_count / 2; i++){
			if(samplesBuffer[i][Q_backup -1] == 0){
				samplesBufferFull=FALSE;
				break;
			}
		}
		
		/*If sample buffer is Full, proceed to make computations
		to determine the type of the object in front of the car*/
		if(samplesBufferFull){
			sleep(1);
			fprintf(stderr,"VALUES BUFFER IS FULL, PROCEED TO CALCULATE...\n");
			lasersMean=0;
			lstdev= 0;
			//Proceed to calculate the mean of each sensor and store it in buffer
			for(i=0; i < (int)processes_count / 2; i++){
				acum= 0.0;
				for(j= 0; j< Q_backup; j++){
					acum+= samplesBuffer[i][j];
					samplesBuffer[i][j]=0;
				}
				means[i]= acum / Q_backup;
				lasersMean+= means[i];
				fprintf(stderr,"||LASER %d mean distance: %.2f||\n",i,means[i]);
			}
			lasersMean= lasersMean / ((float) processes_count / 2);
			for(i=0; i< (int) processes_count / 2; i++){
				lstdev+= pow(means[i] - lasersMean,2);
			}
			lstdev= sqrt(lstdev / ((int) processes_count / 2));
			fprintf(stderr,"||Standard dev: %.2f||\n",lstdev);

			//We always assume that the first laser sensor is the CENTRAL LASER

			fprintf(stderr,"VALUE OF VH TRESHOLD: %.2f\n",T * lstdev);
			fprintf(stderr,"VALUE OF OB TRESHOLD: %.2f\n",W * lstdev);

			isVehicle= TRUE;
			isObstacle= FALSE;

			/*Checking if the difference betweeen edge lasers and central laser
			applies for the vehicle threshold and if the same difference applies
			for obstacle threshold*/
			for(i=1; i< (int) processes_count / 2; i++){
				fprintf(stderr,"DIFFERENCE: %.2f\n",fabs(means[0] - means[i]));
				if(fabs(means[0] - means[i]) > (T * lstdev)){
					isVehicle= FALSE;
				}
				if(fabs(means[0] - means[i]) == (W * lstdev)){
					isObstacle= TRUE;
				}
			}

			if(isVehicle){
				fprintf(stderr,"\n>>>>Object is a VEHICLE<<<<\n");
				continue;
			}
			
			/*Checking if the difference between edge sensors applies for
			obstacle threshold*/
			if(!isObstacle){
				for(i=1; i< (int) processes_count / 2; i++){
					for(j= i + 1; j< (int) processes_count / 2; j++){
						fprintf(stderr,"DIFFERENCE: %.2f\n",fabs(means[i] - means[j]));
						if(fabs(means[i] - means[j]) == (W * lstdev)){
							isObstacle= TRUE;
							break;
						}
					}
				}
			}

			if(isObstacle){
				fprintf(stderr,"\n>>>>Object is a OBSTACLE<<<<\n");
				continue;
			}

			/* If none of them applies, mark the object as unknown */
			if(!isVehicle && !isObstacle){
				fprintf(stderr,"\n<<<<Object is UNKNOWN>>>>\n");
			}

			//Allocating  memory for more or less samples if Q value was changed!
			if(Q_backup != Q){
				for(i=0; i < (int) processes_count / 2; i++){
					free(samplesBuffer[i]);
					samplesBuffer[i]= (float *) calloc(Q,sizeof(float));
				}
				Q_backup= Q;
			}
			
			samplesBufferFull=FALSE;
		}
		/*/Analytics code
		elapsed_time= time(NULL);
		if(elapsed_time - actual_time >=5){
			//fprintf(stderr,"\nOperations done in round %d: %d",round_counter,op_per_sec);
			fprintf(stderr,"\n%d",op_per_sec);
			op_per_sec=0;
			actual_time= time(NULL);
			round_counter++;
		}
		/*/
  	}

	for(i= 0; i < (int) processes_count / 2 ; i++){
		sem_destroy(&buffer_mutexes[i]);
	}
	fclose(logfp);
	fclose(reslogfp);
	return 0;
}



/* =============THREAD/PROCESS HANDLERS=============== */

/*-----Collector routine (Process)-----*/
int sensor_read(key_t shmkey, int proc_index){
	int shmid;
	char *shmem;
  char oldval[SHMSZ];
	
	if ((shmid = shmget(shmkey, SHMSZ, 0666)) < 0) {
  	perror("shmget");
  	return(1);
	}

	if ((shmem = shmat(shmid, NULL, 0)) == (char *) -1) {
  	perror("shmat");
  	return(1);
  }
	
	while(1){
		
    if(strcmp(shmem,oldval)!=0){
			sem_wait(reader_mutexes[proc_index]);
			
			strcpy(sensorData[proc_index],shmem);			//critical section
			strcpy(oldval,shmem);
      
			sem_post(reader_mutexes[proc_index]);
			
    }
		sleep(1);
  }
	
  return(0);
	
}


/*-----Reader routine (Thread)-----*/
void *main_reader(void *param){
	int sensor_index= (int) param;
	int idmem;
	char valueData[SHMSZ];
	char oldDataValue[SHMSZ];
	int i;
	int samples=0;
	idmem= shmem_ids[sensor_index];
	
	while(1){

		if(!readyForSample){
			samples= 0;
		}

		if(readyForSample && samples < Q_backup){
			sem_wait(reader_mutexes[sensor_index]);
			
			strcpy(valueData,sensorData[sensor_index]); //critical section

			sem_post(reader_mutexes[sensor_index]);
			
			
			if(strcmp(oldDataValue,valueData)!=0){
				fprintf(logfp,"Reading from %s sensor %d: %s\n",(sensor_index % 2 == 0 ? "LASER" : "GSCOPE"),idmem,valueData);
				fflush(logfp);
				sem_wait(&buffer_mutexes[(int) sensor_index/2]);

				//If laser data was written
				if(strcmp(valueData,"")!=0 && sensor_index % 2 == 0){
					//store the value of the sensor read in the properly index
					strcpy(distanceBuffer[(int) sensor_index/2][buffd_in[(int) sensor_index/2]],valueData); //critical zone
					buffd_in[(int) sensor_index/2]= (buffd_in[(int) sensor_index/2] + 1) % BUFFER_SIZE;
					//increase sample counter buffer of this sensor
					samplesCounterBuffer[sensor_index]= samplesCounterBuffer[sensor_index] + 1;
					samples++;
				}

				//If gscope data was written
				else if(strcmp(valueData,"")!=0 && sensor_index % 2 != 0){
					strcpy(gscopeBuffer[(int) sensor_index/2][buffg_in[(int) sensor_index/2]],valueData); //critical zone
					buffg_in[(int) sensor_index/2]= (buffg_in[(int) sensor_index/2] + 1) % BUFFER_SIZE;
					//increase sample counter buffer of this sensor
					samplesCounterBuffer[sensor_index]= samplesCounterBuffer[sensor_index] + 1;
					samples++;
				}
			
				sem_post(&buffer_mutexes[(int) sensor_index/2]);

				strcpy(oldDataValue,valueData);			
			}

		}
		sleep(1);
	}
}


/*-----Calculator routine (Thread)-----*/
void *main_calculator(void *param){
	int buffer_index= (int) param;
	float distance;
	float angle;	
	float angles_sum=0;
	float result=0;	
	int num_readings=0;
	char gscopeString[SHMSZ];
	int sampleIndex= 0;
	
	while(1){
		sleep(1);
		sem_wait(&buffer_mutexes[buffer_index]);
		
		distance= atof(distanceBuffer[buffer_index][buffd_out[buffer_index] % BUFFER_SIZE]);
		angle= atof(gscopeBuffer[buffer_index][buffg_out[buffer_index] % BUFFER_SIZE]);
		
		sem_post(&buffer_mutexes[buffer_index]);
		
		//if sample Index reach the limit value, reassing it to zero
		if(samplesBufferFull){
			sampleIndex=0;
		}

		if( distance!=0 && angle!=0){
			result= distance * cos(angle * (M_PI/180));	

			//store the processed value on the samples buffer on properly index
			samplesBuffer[buffer_index][sampleIndex]= result;
			sampleIndex++;
			
			fprintf(reslogfp,"result of Laser %d distance %f and Gscope %d angle %f: %.3f \n",
												shmem_ids[2 * buffer_index],
												distance,
												shmem_ids[2 * buffer_index + 1],
												angle,
												result);
			fflush(reslogfp);			
			buffd_out[buffer_index]= buffd_out[buffer_index] + 1;
			buffg_out[buffer_index]= buffg_out[buffer_index] + 1;
			num_readings++;
			angles_sum+=angle;
			//op_per_sec++;
		}
		else if(distance!=0 && strcmp(gscopeBuffer[buffer_index][buffg_out[buffer_index]],"--")== 0){
			if(num_readings==0){
				result= distance;	
			}
			else{
				result= distance * cos((angles_sum / num_readings) * (M_PI/180));
			}

			//store the processed value on the samples buffer on properly index
			samplesBuffer[buffer_index][sampleIndex]= result;
			sampleIndex++;
			
			fprintf(reslogfp,"result of Laser %d distance %s and Gscope %d angle %f: %f \n",
												shmem_ids[2 * buffer_index],
												distanceBuffer[buffer_index][buffd_out[buffer_index]],
												shmem_ids[2 * buffer_index + 1],
												angles_sum / num_readings,
												result);
			fflush(reslogfp);
			buffd_out[buffer_index]= (buffd_out[buffer_index] + 1) % BUFFER_SIZE;
			buffg_out[buffer_index]= (buffg_out[buffer_index] + 1) % BUFFER_SIZE;
			//op_per_sec++;
		}

	}
	
}

/*-----Changes Monitor routine (Thread)-----*/
void *changes_monitor(void *param){
	time_t oldModifiedTime= time(NULL);
	char * var_token;
	char * values_token;
	char * values[2];
	char * confs_string;
	const char sep[2]= ",";
	const char var_sep[2]= "=";
	char * saveptr1;
	char * saveptr2;
	int fd = -1;
	int i;
	int j;
	long size;
	while(1){
		sleep(2);
		changesfp= fopen("changes.cfg","r");
		//Checking if file exists and we can open it
		if(changesfp != NULL){
			//taking the file descriptor
			fd = fileno(changesfp);
			//taking the file stats
			if(fstat(fd,&fileStat) < 0){
				fprintf(stderr,"@@Cant get changes file info!\n");
				continue;	
			}
			/*if the date of the modified file is greater than last modification
			time (or initial current time in first instance), then the file was 
			modified and we have to get its values*/
			if((fileStat.st_mtime - oldModifiedTime) > 0){
				fprintf(stderr,"** FILE WAS RECENTLY MODIFIED! **\n");
				oldModifiedTime= fileStat.st_mtime;
				
				//Seeking and rewinding pointer to read all once
				fseek(changesfp,0,SEEK_END);
				size= ftell(changesfp);
				rewind(changesfp);
				confs_string= malloc(size + 1);
				fread(confs_string,size,1,changesfp);
				fclose(changesfp);
				
				/*Tokenizing the content of the file
				  The content of file must have the following format:
				  
				  var_name1=val1,var_name2=val2,......var_namen=valn
				  
				  if data does not have this format, it will be ignored
				*/
				var_token= strtok_r(confs_string,sep,&saveptr1);
				for(i=0; var_token != NULL; i++){
						//getting the values of the tokenized variable with its value
						values[0]= strtok_r(var_token,var_sep,&saveptr2);
						values[1]= strtok_r(saveptr2,var_sep,&saveptr2);
						
						if(values[0] != NULL && values[1] != NULL){
							//Checking what variable the data represents
							if(strcmp(values[0],"w")==0 || strcmp(values[0],"W")==0){
								fprintf(stderr,"$$ This value is for obstacle threshold\n\n");
								if(atof(values[1]) > 1){
									W= atof(values[1]);
								}
							}
							else if(strcmp(values[0],"t")==0 || strcmp(values[0],"T")==0){
								fprintf(stderr,"$$ This value is for vehicle threshold\n\n");
								if(atof(values[1]) > 0 && atof(values[1]) < 1){
									T= atof(values[1]);
								}
							}
							else if(strcmp(values[0],"q")==0 || strcmp(values[0],"Q")==0){
								fprintf(stderr,"$$ This value is for no. of samples\n\n");
								if(atoi(values[1]) > 0){
									Q= atoi(values[1]);
								}
							}
							else if(strcmp(values[0],"i")==0 || strcmp(values[0],"I")==0){
								fprintf(stderr,"$$ This value is for sampling interval\n\n");
								if(atoi(values[1]) > 0){
									I= atoi(values[1]);
								}
							}
							else{
								fprintf(stderr,"$$ This value is unknown...ignoring it!\n\n");
							}
						}

						var_token= strtok_r(saveptr1,sep,&saveptr1);
				}
			}
		}
		else{
			fprintf(stdout,"Changes File does not exist!\n");
		}

	}
}



/* =============SIGNAL HANDLERS=============== */

/* --- Handler for SIGCHILD signal ------*/
void sigchild_handler(int sig)
{
  pid_t cpid;
  int status;
  int j;
	int shmkey;

  cpid = waitpid(-1,&status,WNOHANG);
  printf("\n@@@ Process %d exited with status: %d @@@\n", cpid,WEXITSTATUS(status));
  if( WEXITSTATUS(status)==PROC_RST){
    printf("\n======= Restarting Process.... ========\n");
		for(j=0;j<processes_count;j++){
			if(cpid == sensor_processes[j]){
				shmkey= shmem_ids[j];
			}
		}
    recreate_process(&cpid,shmkey);	
  }
  else{
    processes_count--;
    //exit(0);
  }
  
 if(processes_count == 0){
   printf("\n|| NO MORE PROCESSES ALIVE... ||\n");
   //exit(0);
 }
}

/* --- Handler for SIGUSER1 and SIGUSER2 signal --- */
void siguser_handler(int sig){
  if(sig == SIGUSR1){
    printf("\n==> Terminating process %d",getpid());
    exit(PROC_END);
  }
  if(sig == SIGUSR2){
    printf("\n==> Restarting process %d",getpid());
    exit(PROC_RST);
  }
}

/* 
	FUNCTION THAT RESTART THE PROCESS THAT WAS KILLED 
	 
	param oldPid, a pointer the pid of the process that was killed
	param sensorKey, the id of the memshared block of the sensor
*/
int recreate_process(pid_t *oldPid,int sensorKey){
  pid_t cpid;
	int i,j;
	key_t shmemKey;
	int shm_status;
  cpid= fork();
  
	if(cpid<0){
    fprintf(stderr,"\n**An error has ocurred during fork()!**\n");
    return 1;                     
  }

  if(cpid==0)
  {
    printf("\nChild --> pid is %d\n", getpid());
    signal(SIGUSR1,siguser_handler);
    signal(SIGUSR2,siguser_handler);
		shmemKey= sensorKey;
		
		for(i = 0; i < keys_count; i++)
		{
			if(shmemKey = shmem_ids[i]);
			break;
		}
		
		shm_status= sensor_read(shmemKey,i);			
    return shm_status;
  }
   
  if(cpid>0){
		for(j=0;j<processes_count;j++){
			if(*oldPid== sensor_processes[j]){
				sensor_processes[j]= cpid;
				break;
			}
		}
		
  }
	return 0;
}
