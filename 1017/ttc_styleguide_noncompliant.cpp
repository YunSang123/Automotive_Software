#include <stdio.h>
/*
클래스나 함수 : Camelcase
변수 : snake_case


*/


// -------------------------------
// Constants (Thresholds)
// -------------------------------
#define DANGER  1.0f

enum VehicleState {
    STATE_DANGER = 0,
    STATE_CAUTION = 1,
    STATE_SAFE = 2
};

// calculate TTC
float calculate_ttc(float a, float b)
{
    if (b == 0) return 0; // Handle division by zero
    return a / b;
}

// evaluate vehicle state based on TTC
int evaluate_state(float ttc)
{
    if (ttc <= 0) return STATE_SAFE; // Because the relative velocity is zero
    if (ttc < DANGER) return STATE_DANGER; // STATE_DANGER
    if (ttc < 3.0)
        return STATE_CAUTION; // STATE_CAUTION
    else
        return STATE_SAFE; // STATE_SAFE
}

// Global State
int g_state = STATE_SAFE;

// Execute action based on vehicle state
void ExecuteAction(int a, float b, float c)
{
    switch (a) {
        case STATE_SAFE:
        printf("[SAFE] Hold: (v=%.1f km/h, TTC=%.2f)\n", b, c); break;
        case STATE_CAUTION:
        printf("[CAUTION] Adaptive cruise (SCC) engaged: (TTC=%.2f) ---\n", c); break;
        case STATE_DANGER: printf("[DANGER] Emergency braking (AEB): (TTC=%.2f) !!!\n", c); break;
    }
}

// rv : relative velocity
void process_vehicleData(float distance, float vel, float rv)
{
    float ttc = calculate_ttc(distance, rv);
    int New_State = evaluate_state(ttc);

    if (New_State != g_state) {
    printf("\n[STATE CHANGE] %d → %d\n", g_state, New_State);
    g_state = New_State;
    }

    ExecuteAction(g_state, vel, ttc);
}

int main(void)
{    
    //Case1
    process_vehicleData(100.0f, 80.0f, 5.0f);
    // Case2
    process_vehicleData(60.0f, 80.0f, 18.0f);

    // case3
    float ttc3 = calculate_ttc(30.0f, 25.0f);
    int state3 = evaluate_state(ttc3);
    if (state3 != g_state) {
        printf("\n[STATE CHANGE] %d → %d\n", g_state, state3);
        g_state = state3;
    }
    ExecuteAction(g_state, 80.0f, ttc3);

    // CASE4
    float ttc4 = calculate_ttc(40.0f, -10.0f);
    int state4 = evaluate_state(ttc4);
    if (state4 != g_state) 
    {
        printf("\n[STATE CHANGE] %d → %d\n", g_state, state4);
        g_state = state4;
    }
    ExecuteAction(g_state, 80.0f, ttc4);
    return 0;
}