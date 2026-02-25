from libs.simPlant import SimPlant
from KhalilB import DynamicMatrixController
import simpy
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    # ========================================================================
    # PHASE 1: Collect step response data (do this ONCE for all controllers)
    # ========================================================================
    print("=" * 70)
    print("PHASE 1: SYSTEM IDENTIFICATION")
    print("=" * 70)
    
    plant_id = SimPlant(kp=1000, time_constant=0.8, dt=0.01)
    controller_id = DynamicMatrixController(
        plant=plant_id,
        prediction_horizon=5000,  # Use max horizon for identification
        control_horizon=5,
        lambda_reg=0.01
    )

    data_env = simpy.Environment()
    data_env.process(plant_id.run(data_env))
    data_env.process(controller_id.collect_step_response(data_env))
    data_env.run(until=10.0)

    # Extract the step response data to share with all controllers
    shared_step_response = controller_id.step_response.copy()
    
    print(f"Step response collected: {len(shared_step_response)} points")
    print(f"Final value: {shared_step_response[-1]:.2f}\n")

    # ========================================================================
    # PHASE 2: Define different MPC configurations
    # ========================================================================
    mpc_configs = [
        {"name": "(N=100, n_mu=3, λ=1.00)", "N": 100, "n_mu": 3, "lambda": 1.0, "color": "red"},
        {"name": "(N=100, n_mu=3, λ=5.00)", "N": 100, "n_mu": 3, "lambda": 5.0, "color": "green"},
        {"name": "(N=100, n_mu=3, λ=25.00)", "N": 100, "n_mu": 3, "lambda": 25.0, "color": "blue"}
    ]

    # Create setpoint profile
    mpc_duration = 10.0
    dt = 0.01
    num_points = int(mpc_duration / dt)
    step_time = 1.0
    step_index = int(step_time / dt)
    
    setpoint = np.zeros(num_points)
    setpoint[step_index:] = 1000.0

    # ========================================================================
    # PHASE 3: Run each MPC configuration and collect results
    # ========================================================================
    print("=" * 70)
    print("PHASE 2: RUNNING 3 MPC CONFIGURATIONS")
    print("=" * 70)
    
    all_results = []
    
    for i, config in enumerate(mpc_configs):
        print(f"\nRunning MPC {i+1}/{len(mpc_configs)}: {config['name']}")
        
        # Create fresh plant and controller
        plant = SimPlant(kp=1000, time_constant=0.8, dt=dt)
        controller = DynamicMatrixController(
            plant=plant,
            prediction_horizon=config['N'],
            control_horizon=config['n_mu'],
            lambda_reg=config['lambda']
        )
        
        # Share the step response data with this controller
        controller.step_response = shared_step_response.copy()
        controller.build_dynamic_matrix()
        
        # Run MPC simulation
        mpc_env = simpy.Environment()
        mpc_env.process(plant.run(mpc_env))
        mpc_env.process(controller.run_dmc(mpc_env, setpoint))
        mpc_env.process(controller.collect_output(mpc_env, mpc_duration))
        mpc_env.run(until=mpc_duration)
        
        # Store results
        results = controller.get_results()
        results['config'] = config
        all_results.append(results)
        
        # Print performance metrics
        final_output = results['output'][-1]
        sse = abs(final_output - 500.0)
        overshoot = max(0, max(results['output']) - 500.0)
        print(f"  Final output: {final_output:.2f}")
        print(f"  Steady-state error: {sse:.2f}")
        print(f"  Overshoot: {overshoot:.2f}")

    # ========================================================================
    # PHASE 4: Plot comparison of all 5 MPCs
    # ========================================================================
    print("\n" + "=" * 70)
    print("PHASE 3: PLOTTING COMPARISON")
    print("=" * 70)
    
    fig, axes = plt.subplots(2, 1, figsize=(14, 10))
    
    # Plot 1: Output comparison
    for i, results in enumerate(all_results):
        config = results['config']
        axes[0].plot(results['time'], results['output'], 
                    label=config['name'], 
                    color=config['color'],
                    linewidth=2, 
                    alpha=0.8)
    
    # Add setpoint
    setpoint_time = np.arange(0, mpc_duration, dt)
    axes[0].plot(setpoint_time, setpoint, 'k--', 
                label='Setpoint', linewidth=2.5, alpha=0.7)
    
    axes[0].set_xlabel('Time (s)', fontsize=11)
    axes[0].set_ylabel('Output', fontsize=11)
    axes[0].set_title('MPC Controller Comparison - Plant Output', fontsize=13, fontweight='bold')
    axes[0].legend(loc='best', fontsize=9)
    axes[0].grid(True, alpha=0.3)
    axes[0].set_ylim([0, 2000])
    
    # Plot 2: Control signals comparison
    for i, results in enumerate(all_results):
        config = results['config']
        axes[1].plot(results['time'], results['control'], 
                    label=config['name'],
                    color=config['color'],
                    linewidth=2,
                    alpha=0.8)
    
    axes[1].set_xlabel('Time (s)', fontsize=11)
    axes[1].set_ylabel('Control Signal (V)', fontsize=11)
    axes[1].set_title('MPC Controller Comparison - Control Signals', fontsize=13, fontweight='bold')
    axes[1].legend(loc='best', fontsize=9)
    axes[1].grid(True, alpha=0.3)
    axes[1].set_ylim([0, 2])
    
    plt.tight_layout()
    plt.show()
    
    # ========================================================================
    # PHASE 5: Performance summary table
    # ========================================================================
    print("\n" + "=" * 70)
    print("PERFORMANCE SUMMARY")
    print("=" * 70)
    print(f"{'Configuration':<35} {'SSE':>8} {'Overshoot':>10} {'Final':>8}")
    print("-" * 70)
    
    for results in all_results:
        config = results['config']
        final_output = results['output'][-1]
        sse = abs(final_output - 500.0)
        overshoot = max(0, max(results['output']) - 500.0)
        
        print(f"{config['name']:<35} {sse:>8.2f} {overshoot:>10.2f} {final_output:>8.2f}")
    
    print("=" * 70)
