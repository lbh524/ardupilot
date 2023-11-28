#include <AP_HAL/AP_HAL.h>
#include "AP_L1_Control.h"

extern const AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_L1_Control::var_info[] = {
    // @Param: PERIOD
    // @DisplayName: L1 control period
    // @Description: Period in seconds of L1 tracking loop. This parameter is the primary control for agressiveness of turns in auto mode. This needs to be larger for less responsive airframes. The default of 20 is quite conservative, but for most RC aircraft will lead to reasonable flight. For smaller more agile aircraft a value closer to 15 is appropriate, or even as low as 10 for some very agile aircraft. When tuning, change this value in small increments, as a value that is much too small (say 5 or 10 below the right value) can lead to very radical turns, and a risk of stalling.
    // @Units: s
    // @Range: 1 60
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PERIOD",    0, AP_L1_Control, _L1_period, 17),

    // @Param: DAMPING
    // @DisplayName: L1 control damping ratio
    // @Description: Damping ratio for L1 control. Increase this in increments of 0.05 if you are getting overshoot in path tracking. You should not need a value below 0.7 or above 0.85.
    // @Range: 0.6 1.0
    // @Increment: 0.05
    // @User: Advanced
    AP_GROUPINFO("DAMPING",   1, AP_L1_Control, _L1_damping, 0.75f),

    // @Param: XTRACK_I
    // @DisplayName: L1 control crosstrack integrator gain
    // @Description: Crosstrack error integrator gain. This gain is applied to the crosstrack error to ensure it converges to zero. Set to zero to disable. Smaller values converge slower, higher values will cause crosstrack error oscillation.
    // @Range: 0 0.1
    // @Increment: 0.01
    // @User: Advanced
    AP_GROUPINFO("XTRACK_I",   2, AP_L1_Control, _L1_xtrack_i_gain, 0.02),

    // @Param: LIM_BANK
    // @DisplayName: Loiter Radius Bank Angle Limit
    // @Description: The sealevel bank angle limit for a continous loiter. (Used to calculate airframe loading limits at higher altitudes). Setting to 0, will instead just scale the loiter radius directly
    // @Units: deg
    // @Range: 0 89
    // @User: Advanced
    AP_GROUPINFO("LIM_BANK",   3, AP_L1_Control, _loiter_bank_limit, 0.0f),

    AP_GROUPEND
};

//Bank angle command based on angle between aircraft velocity vector and reference vector to path.
//S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
//Proceedings of the AIAA Guidance, Navigation and Control
//Conference, Aug 2004. AIAA-2004-4900.
//Modified to use PD control for circle tracking to enable loiter radius less than L1 length
//Modified to enable period and damping of guidance loop to be set explicitly
//Modified to provide explicit control over capture angle


/*
  Wrap AHRS yaw if in reverse - radians
 */
float AP_L1_Control::get_yaw() const
{
    if (_reverse) {
        return wrap_PI(M_PI + _ahrs.yaw);
    }
    return _ahrs.yaw;
}

/*
  Wrap AHR  S yaw sensor if in reverse - centi-degress
 */
int32_t AP_L1_Control::get_yaw_sensor() const
{
    if (_reverse) {
        return wrap_180_cd(18000 + _ahrs.yaw_sensor);
    }
    return _ahrs.yaw_sensor;
}

/*
  return the bank angle needed to achieve tracking from the last
  update_*() operationo                                
 */
// _latAccDem最终会被用来计算飞机的目标roll—nav_roll_cd，从而将nav_roll_cd作为Roll controller的输入值，进一步通过PID控制器来控制roll角。
int32_t AP_L1_Control::nav_roll_cd(void) const
{
    float ret;
    ret = cosf(_ahrs.pitch)*degrees(atanf(_latAccDem * (1.0f/GRAVITY_MSS)) * 100.0f);
    ret = constrain_float(ret, -9000, 9000);
    return ret;
}

/*
  return the lateral acceleration needed to achieve tracking from the last
  update_*() operation
 */
float AP_L1_Control::lateral_acceleration(void) const
{
    return _latAccDem;  //横向加速度
}

int32_t AP_L1_Control::nav_bearing_cd(void) const  //导航航向角
{
    return wrap_180_cd(RadiansToCentiDegrees(_nav_bearing));  //弧度值转换为百分度
}

int32_t AP_L1_Control::bearing_error_cd(void) const
{
    return RadiansToCentiDegrees(_bearing_error);
}

int32_t AP_L1_Control::target_bearing_cd(void) const
{
    return wrap_180_cd(_target_bearing_cd);
}

/*
  this is the turn distance assuming a 90 degree turn  90°下的转弯距离
 */
float AP_L1_Control::turn_distance(float wp_radius) const
{
    wp_radius *= sq(_ahrs.get_EAS2TAS());    //wp_radius乘以等效空速比  wp_radius航点半径
    return MIN(wp_radius, _L1_dist);
}

/*
  this approximates the turn distance for a given turn angle. If the
  turn_angle is > 90 then a 90 degree turn distance is used, otherwise
  the turn distance is reduced linearly.
  This function allows straight ahead mission legs to avoid thinking
  they have reached the waypoint early, which makes things like camera
  trigger and ball drop at exact positions under mission control much easier

  该函数用于近似计算给定转弯角度的转弯距离。如果转弯角度大于90度，则使用90度的转弯距离；否则，转弯距离将线性减小。
   这个函数的目的是为了让直线航段能够避免过早地认为已经到达航点，从而使得在任务控制下的摄像头触发和球的投放等精确位置操作更加容易
 */
float AP_L1_Control::turn_distance(float wp_radius, float turn_angle) const   //转弯距离
{
    float distance_90 = turn_distance(wp_radius);
    turn_angle = fabsf(turn_angle);
    if (turn_angle >= 90) {
        return distance_90;
    }
    return distance_90 * turn_angle / 90.0f;
}

float AP_L1_Control::loiter_radius(const float radius) const
{
    // prevent an insane loiter bank limit
    float sanitized_bank_limit = constrain_float(_loiter_bank_limit, 0.0f, 89.0f);  // 进行范围约束，将其限制在 0.0 到 89.
    float lateral_accel_sea_level = tanf(radians(sanitized_bank_limit)) * GRAVITY_MSS;  //通过计算得到在海平面上的水平加速度

    float nominal_velocity_sea_level = 0.0f;
    if(_tecs != nullptr) {
        nominal_velocity_sea_level =  _tecs->get_target_airspeed(); //如果存在 _tecs 对象，则获取目标空速作为 nominal_velocity_sea_level
    }

    float eas2tas_sq = sq(_ahrs.get_EAS2TAS());  //计算 eas2tas_sq，即 Equivalent Airspeed Squared 的平方，等效空速

    if (is_zero(sanitized_bank_limit) || is_zero(nominal_velocity_sea_level) ||
        is_zero(lateral_accel_sea_level)) {
        // Missing a sane input for calculating the limit, or the user has
        // requested a straight scaling with altitude. This will always vary
        // with the current altitude, but will at least protect the airframe  
        // 该条件判断是判断在计算环绕半径时是否缺少了合理的输入，或者用户是否要求按照海拔高度直接进行缩放。
        // 这种计算方式会随着当前海拔高度的变化而变化，但至少可以保护飞行器。
        return radius * eas2tas_sq;
    } else {
        float sea_level_radius = sq(nominal_velocity_sea_level) / lateral_accel_sea_level;  //
        if (sea_level_radius > radius) {
            // If we've told the plane that its sea level radius is unachievable fallback to
            // straight altitude scaling    
            // 如果 sea_level_radius 大于用户指定的半径 radius，表示飞机无法实现在海平面上的环绕半径，这种情况下代码会回退到直接按照海拔高度缩放的方式，返回 radius * eas2tas_sq
            return radius * eas2tas_sq;
        } else {
            // select the requested radius, or the required altitude scale, whichever is safer
            return MAX(sea_level_radius * eas2tas_sq, radius);
        }
    }
}

bool AP_L1_Control::reached_loiter_target(void)
{
    return _WPcircle;
}

/**
   prevent indecision in our turning by using our previous turn
   decision if we are in a narrow angle band pointing away from the
   target and the turn angle has changed sign
   如果我们在一个狭窄的角度带指向目标，并且转弯角度已经改变标志，通过使用我们之前的转弯决定来防止我们在转弯时的犹豫不决
 */
void AP_L1_Control::_prevent_indecision(float &Nu)  //在特定条件下进行转弯决策的调整，以提高飞行稳定性和减少不确定性。
{
    const float Nu_limit = 0.9f*M_PI;   // 
    if (fabsf(Nu) > Nu_limit &&
        fabsf(_last_Nu) > Nu_limit &&
        labs(wrap_180_cd(_target_bearing_cd - get_yaw_sensor())) > 12000 &&
        Nu * _last_Nu < 0.0f) {
        // we are moving away from the target waypoint and pointing
        // away from the waypoint (not flying backwards). The sign
        // of Nu has also changed, which means we are
        // oscillating in our decision about which way to go
        // 表示当前飞机正在远离目标航点并且指向偏离航点的方向（不是向后飞行）。而且 Nu 的符号已经改变，这意味着我们在决定要转向哪个方向时出现了振荡
        Nu = _last_Nu;
    }
}

// update L1 control for waypoint navigation  更新L1 control的航点
void AP_L1_Control::update_waypoint(const struct Location &prev_WP, const struct Location &next_WP, float dist_min)
{

    struct Location _current_loc;
    float Nu;
    float xtrackVel;
    float ltrackVel;

    // 初始化或更新L1控制器
    uint32_t now = AP_HAL::micros();  // 获取当前时间变量
    float dt = (now - _last_update_waypoint_us) * 1.0e-6f; // 计算从上次更新航点以来经过的时间，将其保存在变量 dt 中，单位 us
    if (dt > 1) {
        // controller hasn't been called for an extended period of
        // time.  Reinitialise it.  如果经过的时间超过 1 秒，表示控制器长时间没有被调用，需要重新初始化控制器
        _L1_xtrack_i = 0.0f;
    }
    if (dt > 0.1) {  // 对 dt 进行判断，如果超过了 0.1 秒，则将 dt 设置为 0.1 秒；这个判断用于限制每次更新的时间间隔，以避免时间间隔过长导致控制器不稳定。
        dt = 0.1; 
    }
    _last_update_waypoint_us = now;

    // Calculate L1 gain required for specified damping  计算指定阻尼所需的L1增益
    float K_L1 = 4.0f * _L1_damping * _L1_damping;  //     // L1 tracking loop damping ratio  L1跟踪环路阻尼比

    // Get current position and velocity    获取当前飞机的位置
    if (_ahrs.get_location(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing   _data_is_stale 设置为 true，表示数据已过时，并直接返回。
        _data_is_stale = true;
        return;
    }

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();  //地速向量，二维表示地速的大小和方向

    // update _target_bearing_cd 更新目标方位角
    _target_bearing_cd = _current_loc.get_bearing_to(next_WP);

    //Calculate groundspeed  计算地速
    float groundSpeed = _groundspeed_vector.length();
    if (groundSpeed < 0.1f) {
        // use a small ground speed vector in the right direction,
        // allowing us to use the compass heading at zero GPS velocity
        groundSpeed = 0.1f;
        _groundspeed_vector = Vector2f(cosf(get_yaw()), sinf(get_yaw())) * groundSpeed;
    }

    // Calculate time varying control parameters  计算时变控制参数
    // Calculate the L1 length required for specified period  根据_L1_period计算特定L1长度
    // 0.3183099 = 1/pi    L1_dist = 1/π * damping * period * speed ≈ 0.3183099* damping * period * speed 
    _L1_dist = MAX(0.3183099f * _L1_damping * _L1_period * groundSpeed, dist_min);

    // Calculate the NE position of WP B relative to WP A  计算航点A与B的北东位置长度
    Vector2f AB = prev_WP.get_distance_NE(next_WP);
    float AB_length = AB.length();

    // Check for AB zero length and track directly to the destination
    // if too small  如果AB距离很短，那么就直接将B点记作当前目标点，径直朝向B
    if (AB.length() < 1.0e-6f) {
        AB = _current_loc.get_distance_NE(next_WP);
        if (AB.length() < 1.0e-6f) {
            AB = Vector2f(cosf(get_yaw()), sinf(get_yaw()));
        }
    }
    AB.normalize();  //向量 AB 进行标准化处理，将其转换为单位向量（长度为1）

    // Calculate the NE position of the aircraft relative to WP A   计算航点A到飞机的北东距离长度信息
    const Vector2f A_air = prev_WP.get_distance_NE(_current_loc);

    // calculate distance to target track, for reporting  计算航迹跟踪误差，|AC|*sin(AB与AC的夹角)   即偏离航线的距离
    _crosstrack_error = A_air % AB;

    //Determine if the aircraft is behind a +-135 degree degree arc centred on WP A
    //and further than L1 distance from WP A. Then use WP A as the L1 reference point
    //Otherwise do normal L1 guidance
    float WP_A_dist = A_air.length();// 向量AC的长度 |AC|
    float alongTrackDist = A_air * AB;   //|AC|*cos(AB与AC的夹角)   即AC在AB上的投影
    if (WP_A_dist > _L1_dist && alongTrackDist/MAX(WP_A_dist, 1.0f) < -0.7071f)
    // |AC|*cos(AB与AC的夹角)/|AC|=alongTrackDist/MAX(WP_A_dist, 1.0f)
    {
        //|AC|长度大于_L1_dist 且 AC和AB之间的夹角在+-135度之外  使用A点作为L1的参考点
        //Calc Nu to fly To WP A
        Vector2f A_air_unit = (A_air).normalized(); // Unit vector from WP A to aircraft  向量AC的单位向量
        xtrackVel = _groundspeed_vector % (-A_air_unit); // Velocity across line  垂直AC向量方向的速度 
        ltrackVel = _groundspeed_vector * (-A_air_unit); // Velocity along line  平行于AC向量方向的速度
        Nu = atan2f(xtrackVel,ltrackVel);  //地速与AC向量之间的夹角
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else if (alongTrackDist > AB_length + groundSpeed*3) {
        // we have passed point B by 3 seconds. Head towards B
        // Calc Nu to fly To WP B  飞行超过B点3秒，要飞回B点，计算飞往B点的Nu（与飞往A点的计算方法一致）
        const Vector2f B_air = next_WP.get_distance_NE(_current_loc);  //计算BC向量
        Vector2f B_air_unit = (B_air).normalized(); // Unit vector from WP B to aircraft  单位化BC向量
        xtrackVel = _groundspeed_vector % (-B_air_unit); // Velocity across line  垂直BC向量方向的速度 
        ltrackVel = _groundspeed_vector * (-B_air_unit); // Velocity along line 平行于BC向量方向的速度
        Nu = atan2f(xtrackVel,ltrackVel);
        _nav_bearing = atan2f(-B_air_unit.y , -B_air_unit.x); // bearing (radians) from AC to L1 point
    } else { //Calc Nu to fly along AB line 否则使用正常的L1引导

        //Calculate Nu2 angle (angle of velocity vector relative to line connecting waypoints)  计算飞往AB点轨迹线的Nu Nu2为速度与轨迹线之间的夹角
        xtrackVel = _groundspeed_vector % AB; // Velocity cross track 垂直AB向量方向的速度 
        ltrackVel = _groundspeed_vector * AB; // Velocity along track 平行于AB向量方向的速度 
        float Nu2 = atan2f(xtrackVel,ltrackVel);  //地速与AB向量之间的夹角
        //Calculate Nu1 angle (Angle to L1 reference point)  Nu1为飞机与L1点连线和AB轨迹线之间的夹角
        float sine_Nu1 = _crosstrack_error/MAX(_L1_dist, 0.1f);  //计算sin(Nu1)
        //Limit sine of Nu1 to provide a controlled track capture angle of 45 deg   //限定Nu1在45度内
        sine_Nu1 = constrain_float(sine_Nu1, -0.7071f, 0.7071f);  //反三角函数计算Nu1
        float Nu1 = asinf(sine_Nu1);

        // compute integral error component to converge to a crosstrack of zero when traveling
        // straight but reset it when disabled or if it changes. That allows for much easier
        // tuning by having it re-converge each time it changes.
        //如果积分参数小于零或改变，则复位积分误差值
        if (_L1_xtrack_i_gain <= 0 || !is_equal(_L1_xtrack_i_gain.get(), _L1_xtrack_i_gain_prev)) {
            _L1_xtrack_i = 0;
            _L1_xtrack_i_gain_prev = _L1_xtrack_i_gain;
        } else if (fabsf(Nu1) < radians(5)) {          //Nu1小于5度
            _L1_xtrack_i += Nu1 * _L1_xtrack_i_gain * dt;

            // an AHRS_TRIM_X=0.1 will drift to about 0.08 so 0.1 is a good worst-case to clip at
            //AHRS_TRIM_X=0.1会漂移到0.08，所以0.1是最坏的情况  
            _L1_xtrack_i = constrain_float(_L1_xtrack_i, -0.1f, 0.1f);//限幅
        }

        // to converge to zero we must push Nu1 harder  //为了收敛到零，我们必须更努力地推动Nu1
        Nu1 += _L1_xtrack_i;

        Nu = Nu1 + Nu2;
        _nav_bearing = wrap_PI(atan2f(AB.y, AB.x) + Nu1);   // bearing (radians) from AC to L1 point  wrap用于将给定的弧度值限制在 [-π, π] 范围内
    }
     //如果我们在一个狭窄的角度范围内，并且转弯角度已经改变，可以使用之前的转弯决定来防止转弯时的犹豫不决
    _prevent_indecision(Nu);
    _last_Nu = Nu;

    //Limit Nu to +-(pi/2)  //限定Nu在+-90°
    Nu = constrain_float(Nu, -1.5708f, +1.5708f);
    _latAccDem = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);  // latAccDem = 4 * damping² * speed² * sin(Nu) / L1_dist .(Nu = Nu1 + Nu2)

    // Waypoint capture status is always false during waypoint following    //在跟踪Waypoint期间，Waypoint捕获状态始终为false
    _WPcircle = false;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for loitering
void AP_L1_Control::update_loiter(const struct Location &center_WP, float radius, int8_t loiter_direction)
{
    struct Location _current_loc;

    // scale loiter radius with square of EAS2TAS to allow us to stay
    // stable at high altitude 
    radius = loiter_radius(fabsf(radius));  
    // 计算loiter半径

    // Calculate guidance gains used by PD loop (used during circle tracking)  计算PD控制增益(用于跟踪圆圈)
    float omega = (6.2832f / _L1_period);
    float Kx = omega * omega;
    float Kv = 2.0f * _L1_damping * omega;

    // Calculate L1 gain required for specified damping (used during waypoint capture)  计算L1增益(用于捕获航点)
    float K_L1 = 4.0f * _L1_damping * _L1_damping;

    //Get current position and velocity  获取当前飞机的位置和速度
    if (_ahrs.get_location(_current_loc) == false) {
        // if no GPS loc available, maintain last nav/target_bearing
        _data_is_stale = true;
        return;
    }

    // 通过融合空速计和GPS二者的速度，计算出地速
    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed  计算地速
    float groundSpeed = MAX(_groundspeed_vector.length() , 1.0f);


    // update _target_bearing_cd  计算目标航向角
    _target_bearing_cd = _current_loc.get_bearing_to(center_WP);


    // Calculate time varying control parameters
    // Calculate the L1 length required for specified period  
    // 计算L1 length
    // 0.3183099 = 1/pi
    _L1_dist = 0.3183099f * _L1_damping * _L1_period * groundSpeed;

    //Calculate the NE position of the aircraft relative to WP A      // 计算飞机相对航点A的水平位置向量
    const Vector2f A_air = center_WP.get_distance_NE(_current_loc);

    // Calculate the unit vector from WP A to aircraft
    // protect against being on the waypoint and having zero velocity
    // if too close to the waypoint, use the velocity vector
    // if the velocity vector is too small, use the heading vector
    Vector2f A_air_unit;    // 计算A_air的单位长度
    if (A_air.length() > 0.1f) {
        A_air_unit = A_air.normalized();
    } else {
        if (_groundspeed_vector.length() < 0.1f) {
            A_air_unit = Vector2f(cosf(_ahrs.yaw), sinf(_ahrs.yaw));
        } else {
            A_air_unit = _groundspeed_vector.normalized();
        }
    }

    //Calculate Nu to capture center_WP   // 计算捕获中心航点的Nu
    float xtrackVelCap = A_air_unit % _groundspeed_vector; // Velocity across line - perpendicular to radial inbound to WP  指向航点的垂直方向速度
    float ltrackVelCap = - (_groundspeed_vector * A_air_unit); // Velocity along line - radial inbound to WP  指向航点的速度
    float Nu = atan2f(xtrackVelCap,ltrackVelCap);

    _prevent_indecision(Nu);
    _last_Nu = Nu;

    Nu = constrain_float(Nu, -M_PI_2, M_PI_2); //Limit Nu to +- Pi/2

    //Calculate lat accln demand to capture center_WP (use L1 guidance law)  // 用L1制导律计算水平倾斜加速度
    float latAccDemCap = K_L1 * groundSpeed * groundSpeed / _L1_dist * sinf(Nu);  

    //Calculate radial position and velocity errors  // 计算圆圈半径和速度的误差
    float xtrackVelCirc = -ltrackVelCap; // Radial outbound velocity - reuse previous radial inbound velocity  径向离开环绕圈的速度（环绕圈半径方向）
    float xtrackErrCirc = A_air.length() - radius; // Radial distance from the loiter circle

    // keep crosstrack error for reporting   记录航迹误差
    _crosstrack_error = xtrackErrCirc;

    //Calculate PD control correction to circle waypoint_ahrs.roll  用PD控制器计算水平倾斜加速度
    float latAccDemCircPD = (xtrackErrCirc * Kx + xtrackVelCirc * Kv);

    //Calculate tangential velocity  计算切向速度
    float velTangent = xtrackVelCap * float(loiter_direction);

    //Prevent PD demand from turning the wrong way by limiting the command when flying the wrong way
    if (ltrackVelCap < 0.0f && velTangent < 0.0f) {
        latAccDemCircPD =  MAX(latAccDemCircPD, 0.0f);
    }

    // Calculate centripetal acceleration demand  计算向心加速度
    float latAccDemCircCtr = velTangent * velTangent / MAX((0.5f * radius), (radius + xtrackErrCirc));

    //Sum PD control and centripetal acceleration to calculate lateral manoeuvre demand   P+D+ff
    float latAccDemCirc = loiter_direction * (latAccDemCircPD + latAccDemCircCtr);

    // Perform switchover between 'capture' and 'circle' modes at the
    // point where the commands cross over to achieve a seamless transfer
    // Only fly 'capture' mode if outside the circle
    /*
       可以发现该函数包含两个控制器，①capture模式—L1控制器 ②circle模式—PD控制器，两个控制器都会计算出飞机的水平加速度的目标值，
       即_latAccDem。这两个控制器的工作范围不同，当飞机还没有跟踪上圆圈，也就是还在圆圈外环的时候用L1控制器，
       当飞机已经跟踪上了圆圈的时候用PD控制器。
    */
    if (xtrackErrCirc > 0.0f && loiter_direction * latAccDemCap < loiter_direction * latAccDemCirc) {
        _latAccDem = latAccDemCap;
        _WPcircle = false;
        _bearing_error = Nu; // angle between demanded and achieved velocity vector, +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians) from AC to L1 point
    } else {
        _latAccDem = latAccDemCirc;
        _WPcircle = true;
        _bearing_error = 0.0f; // bearing error (radians), +ve to left of track
        _nav_bearing = atan2f(-A_air_unit.y , -A_air_unit.x); // bearing (radians)from AC to L1 point
    }

    _data_is_stale = false; // status are correctly updated with current waypoint data
}


// update L1 control for heading hold navigation  更新 L1 控制器以进行航向保持导航的函数
void AP_L1_Control::update_heading_hold(int32_t navigation_heading_cd)
{
    // Calculate normalised frequency for tracking loop
    const float omegaA = 4.4428f/_L1_period; // sqrt(2)*pi/period
    // Calculate additional damping gain

    int32_t Nu_cd;
    float Nu;

    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = wrap_180_cd(navigation_heading_cd);
    _nav_bearing = radians(navigation_heading_cd * 0.01f);

    Nu_cd = _target_bearing_cd - wrap_180_cd(_ahrs.yaw_sensor);
    Nu_cd = wrap_180_cd(Nu_cd);
    Nu = radians(Nu_cd * 0.01f);

    Vector2f _groundspeed_vector = _ahrs.groundspeed_vector();

    //Calculate groundspeed
    float groundSpeed = _groundspeed_vector.length();

    // Calculate time varying control parameters
    _L1_dist = groundSpeed / omegaA; // L1 distance is adjusted to maintain a constant tracking loop frequency
    float VomegaA = groundSpeed * omegaA;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _crosstrack_error = 0;

    _bearing_error = Nu; // bearing error angle (radians), +ve to left of track

    // Limit Nu to +-pi
    Nu = constrain_float(Nu, -M_PI_2, M_PI_2);
    _latAccDem = 2.0f*sinf(Nu)*VomegaA;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}

// update L1 control for level flight on current heading  更新 L1 控制器以进行当前航向上的水平飞行的函数
void AP_L1_Control::update_level_flight(void)
{
    // copy to _target_bearing_cd and _nav_bearing
    _target_bearing_cd = _ahrs.yaw_sensor;
    _nav_bearing = _ahrs.yaw;
    _bearing_error = 0;
    _crosstrack_error = 0;

    // Waypoint capture status is always false during heading hold
    _WPcircle = false;

    _latAccDem = 0;

    _data_is_stale = false; // status are correctly updated with current waypoint data
}
