#ifndef IPC_STRUCT_H
#define IPC_STRUCT_H

typedef struct 
{
	// radius is in m, and speed is in m/s
	// negative radius is left hand side curve, positive is right hand side
	double radius;
	double speed;
}Vehicle_vector_t;



#endif
