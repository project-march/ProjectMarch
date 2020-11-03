// References
// [1] https://github.com/uzh-rpg/rpg_mpc/blob/master/model/quadrotor_model_thrustrates.cpp
// [2] https://github.com/ethz-asl/mav_control_rw

#define _USE_MATH_DEFINES

#include <iostream>
#include <acado_toolkit.hpp>
#include <acado_code_generation.hpp>
#include <acado_gnuplot.hpp>
#include <cmath>
#include <chrono> 

using namespace std;
using namespace std::chrono;

USING_NAMESPACE_ACADO

int main( ){

	/*
  	Switch between code generation and analysis.
  	If CODE_GEN is true the system is compiled into an optimizaiton problem
  	for real-time iteration and all code to run it online is generated.
  	Constraints and reference structure is used but the values will be set on
  	runtinme.
  	If CODE_GEN is false, the system is compiled into a standalone optimization
  	and solved on execution. The reference and constraints must be set in here.
  	*/

	const bool CODE_GEN = true;

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState   x1;
	DifferentialState   x2;

	Control             u;

	// Parameters with exemplary values
    const double t_start = 0.0;                // Initial time [s]
    const double t_end = 0.2;                  // Time horizon [s]
	const double t_sim = 8.0;                  // Simulation time [s]
	const double dt = 0.04;                    // Discretization time [s]
	const int N = round(t_end/dt);             // Number of nodes
	const double g = 9.81;                     // Gravity is everywhere [m/s^2]
	const double m = 3.0;                      // Point mass [kg]
	const double L = 0.60;                     // Arm length [m]

	const double angle_min = -90*(M_PI/180);   // Minimal motor angle [rad]
	const double angle_max = 90*(M_PI/180);    // Maximal motor angle [rad]
	const double T_min = -50;                  // Minimal torque [Nm]
	const double T_max = 50;                   // Maximum torque [Nm]


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

    // System Dynamics (linear)
    // f << dot(x1) == x2;
    // f << dot(x2) == -(g/L) + u/(m*L*L);

    // System Dynamics (nonlinear)
    f << dot(x1) == x2;
    f << dot(x2) == -(g/L)*cos(x1) + u/(m*L*L);


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << x1;
    h << x2;
    h << u;

    Function hN;

    hN << x1;
    hN << x2;

    DMatrix Q(h.getDim(), h.getDim());
    Q.setIdentity();
    Q(0,0) = 100; // x1
    Q(1,1) = 1; // x2
    Q(2,2) = 0.01; // u

    DMatrix QN(hN.getDim(), hN.getDim());
    QN.setIdentity();
    QN(0,0) = Q(0,0); // x1
    QN(1,1) = Q(1,1); // x2

    DVector r(h.getDim());
    r.setZero();
    r(2) = m*L*g;

    DVector rN(hN.getDim());
    rN.setZero();


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, N );

    if(!CODE_GEN) {

    ocp.minimizeLSQ( Q, h, r );
    ocp.minimizeLSQEndTerm( QN, hN, rN );

    } else {

    ocp.minimizeLSQ( Q, h );
    ocp.minimizeLSQEndTerm( QN, hN );

    }

    ocp.subjectTo( f );

    ocp.subjectTo(angle_min <= x1 <= angle_max);
    ocp.subjectTo(T_min <= u <= T_max);


    if(!CODE_GEN) {

        // SETTING UP THE (SIMULATED) PROCESS:
        // -----------------------------------
        OutputFcn identity;
        DynamicSystem dynamicSystem( f, identity );

        Process process( dynamicSystem, INT_RK45 );


        // SETTING UP THE MPC CONTROLLER:
        // ------------------------------
        RealTimeAlgorithm alg( ocp,dt );
        alg.set( MAX_NUM_ITERATIONS, 2 );

        // StaticReferenceTrajectory zeroReference;
        StaticReferenceTrajectory reference( "References/refSin.txt" );

        Controller controller( alg,reference );
  	

        // SETTING UP THE SIMULATION ENVIRONMENT,  RUN THE EXAMPLE...
        // ----------------------------------------------------------
        SimulationEnvironment sim( t_start,t_sim,process,controller );

        DVector x0(f.getDim());
        x0(0) = 0.00;
        x0(1) = 0.00;

        if (sim.init( x0 ) != SUCCESSFUL_RETURN)
          exit( EXIT_FAILURE );
        if (sim.run( ) != SUCCESSFUL_RETURN)
          exit( EXIT_FAILURE );


		// ... AND PLOT THE RESULTS
  	    // ------------------------
		VariablesGrid sampledProcessOutput;
		sim.getSampledProcessOutput( sampledProcessOutput );

		VariablesGrid feedbackControl;
		sim.getFeedbackControl( feedbackControl ); 

		GnuplotWindow window( PLOT_AT_EACH_ITERATION );
		window.addSubplot( sampledProcessOutput(0),  "Angle [rad]" );
		window.addSubplot( sampledProcessOutput(1),  "Velocity [rad/s]" );
		window.addSubplot( feedbackControl(0),       "Control input [N/m]" );
		window.plot();

        // sampledProcessOutput(0).print("log_states.txt");
        // feedbackControl(0).print("log_control.txt");

        return EXIT_SUCCESS;
	} 
    else {
		// For code generation, we can set some properties.
        // The main reason for a setting is given as comment.
        OCPexport mpc(ocp);

        mpc.set(HESSIAN_APPROXIMATION,              GAUSS_NEWTON);          // is robust, stable
        mpc.set(DISCRETIZATION_TYPE,                MULTIPLE_SHOOTING);     // good convergence
        mpc.set(SPARSE_QP_SOLUTION,                 FULL_CONDENSING_N2);    // due to qpOASES
        mpc.set(INTEGRATOR_TYPE,                    INT_IRK_GL4);           // accurate
        mpc.set(NUM_INTEGRATOR_STEPS,               N);
        mpc.set(QP_SOLVER,                          QP_QPOASES);            // free, source code
        mpc.set(HOTSTART_QP,                        YES);
        mpc.set(CG_USE_OPENMP,                      YES);                   // paralellization
        mpc.set(CG_HARDCODE_CONSTRAINT_VALUES,      NO);                    // set on runtime
        mpc.set(CG_USE_VARIABLE_WEIGHTING_MATRIX,   NO);                    // time-varying costs
        mpc.set( USE_SINGLE_PRECISION,              YES);                   // Single precision

        // Generate test and make files
        mpc.set( GENERATE_TEST_FILE,                NO);
        mpc.set( GENERATE_MAKE_FILE,                NO);

        // Export everything.
        if(mpc.exportCode("../src/mpc_codegen") != SUCCESSFUL_RETURN)
            exit( EXIT_FAILURE );
            mpc.printDimensionsQP( );
        }

	return EXIT_SUCCESS;
}