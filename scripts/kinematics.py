import numpy as np

def create_target_pose(position, orientation_rpy):
    
    x, y, z = position # X, Y, Z coordinates
    rx, ry, rz = orientation_rpy # Roll, Pitch, Yaw
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(rx), -np.sin(rx)],
        [0, np.sin(rx),  np.cos(rx)]
    ])
    
    Ry = np.array([
        [np.cos(ry), 0, np.sin(ry)],
        [0, 1, 0],
        [-np.sin(ry), 0, np.cos(ry)]
    ])
    
    Rz = np.array([
        [np.cos(rz), -np.sin(rz), 0],
        [np.sin(rz),  np.cos(rz), 0],
        [0, 0, 1]
    ])
    R_3x3 = Rz @ Ry @ Rx # Rotation matrix from roll-pitch-yaw
    
    T_4x4 = np.eye(4)
    T_4x4[0:3, 0:3] = R_3x3 # Add rotation part
    T_4x4[0:3, 3] = [x, y, z] # Add translation part
    
    return T_4x4

def dh_matrix(theta, d, a, alpha):
    """
    Create the Denavit-Hartenberg transformation matrix.
    """
    cos_t = np.cos(theta)
    sin_t = np.sin(theta)
    cos_a = np.cos(alpha)
    sin_a = np.sin(alpha)

    # Ma trận T từ (i-1) đến i
    T = np.array([
        [cos_t, -sin_t * cos_a,  sin_t * sin_a, a * cos_t],
        [sin_t,  cos_t * cos_a, -cos_t * sin_a, a * sin_t],
        [0    ,  sin_a        ,  cos_a        , d        ],
        [0    ,  0            ,  0            , 1        ]
    ])
    return T

def forward_kinematics(thetas, d,a, alpha):
    
    T = np.eye(4)
    for i in range(6):
        T_i = dh_matrix(thetas[i], d[i], a[i], alpha[i])
        T = np.dot(T, T_i)

    return T

def forward_kinematics_UR5(thetas, d, a):
    """
    Hard code for UR5 robot forward kinematics using DH parameters.
    """
    t1, t2, t3, t4, t5, t6 = thetas
    d1, d4, d5, d6 = d
    a2, a3 = a
    pi_2 = np.pi / 2

    # Calculate individual transformation matrices
    T0_1 = dh_matrix(t1, d1,  0,   pi_2)
    T1_2 = dh_matrix(t2,  0, a2,   0)
    T2_3 = dh_matrix(t3,  0, a3,   0)
    T3_4 = dh_matrix(t4, d4,  0,   pi_2)
    T4_5 = dh_matrix(t5, d5,  0,  -pi_2)
    T5_6 = dh_matrix(t6, d6,  0,   0)

    # Matrix Multiplication to get T0_6
    T0_6 = T0_1 @ T1_2 @ T2_3 @ T3_4 @ T4_5 @ T5_6
    
    return T0_6

#Inverse Kinematics

"""Find first three joint angles (theta1, theta2, theta3) based on wrist center position."""

def find_wrist_center(T_target, d6):

    P_e = T_target[0:3,3] # Target position of end-effector, take column 4 and row 0 to 2
    a = T_target[0:3,2] # Z axis of end-effector, take column 3 and row 0 to 2
    P_c = P_e - d6 * a # Wrist center position
    return P_c

def solve_theta1(P_c, d4):

    x_c, y_c, z_c = P_c
    d_xy = np.sqrt(x_c**2 + y_c**2)
    if d_xy < d4:
        raise ValueError("Wrist center is too close to the base axis; no valid theta1.")
    
    phi = np.arctan2(y_c, x_c)
    psi = np.arctan2(np.sqrt(d_xy**2 - d4**2),d4)

    #Standard DH needs to add pi/2
    theta1_1 = phi + psi + np.pi/2
    theta1_2 = phi - psi + np.pi/2

    return theta1_1, theta1_2

def solve_theta3(theta1, P_c, d1, a2, a3):
    theta3 = []

    for i in range(len(theta1)):
        current_t1 = theta1[i]

        x_c, y_c, z_c = P_c
        s = x_c * np.cos(current_t1) + y_c * np.sin(current_t1)
        h = z_c - d1
        D = np.sqrt(s**2 + h**2)

        num = D**2 - a2**2 - a3**2
        den = 2 * a2 * a3

        if den == 0:
            raise ValueError("Denominator in theta3 calculation is zero; no valid theta3.")
        cos_theta3 = num / den
        if abs(cos_theta3) > 1:
            raise ValueError("No valid theta3 found; cos(theta3) out of range.")
    
        theta3_pos = np.arccos(cos_theta3)
        theta3_neg = -np.arccos(cos_theta3)
        theta3.append((theta3_pos, theta3_neg))
        
    return theta3

def solve_theta2(theta1, theta3, P_c, d1, a2, a3):
    theta2 = []

    for i in range(len(theta1)):
        current_t1 = theta1[i]
        current_t3_pos, current_t3_neg = theta3[i]

        x_c, y_c, z_c = P_c
        s = x_c * np.cos(current_t1) + y_c * np.sin(current_t1)
        h = z_c - d1
        D = np.sqrt(s**2 + h**2)

        # For positive theta3
        k1_pos = a2 + a3 * np.cos(current_t3_pos)
        k2_pos = a3 * np.sin(current_t3_pos)
        theta2_pos = np.arctan2(h, s) - np.arctan2(k2_pos, k1_pos)

        # For negative theta3
        k1_neg = a2 + a3 * np.cos(current_t3_neg)
        k2_neg = a3 * np.sin(current_t3_neg)
        theta2_neg = np.arctan2(h, s) - np.arctan2(k2_neg, k1_neg)

        theta2.append((theta2_pos, theta2_neg))

    return theta2

"""Find last three joint angles (theta4, theta5, theta6) based on orientation of end-effector and first three joint angles."""

def solve_wrist_matrix(theta1, theta2, theta3, T_target, d1, a2, a3, d4, d5, d6):
    # Compute T0_3
    T0_1 = dh_matrix(theta1, d1,  0,   np.pi/2)
    T1_2 = dh_matrix(theta2,  0, a2,   0)
    T2_3 = dh_matrix(theta3,  0, a3,   0)
    T0_3 = T0_1 @ T1_2 @ T2_3

    # Compute T3_6
    T3_6 = np.linalg.inv(T0_3) @ T_target

    return T3_6

def solve_theta5(M):

    theta5_1 = np.arccos(M[2,2])
    theta5_2 = - np.arccos(M[2,2])  

    return [theta5_1, theta5_2]

def solve_theta4_theta6(M, theta5):
    theta4 =[]
    theta6 =[]
    for current_theta5 in theta5:

        sin_t5 = np.sin(current_theta5)
        if abs(sin_t5) < 1e-6:
            raise ValueError("Singularity encountered; theta5 is 0 or pi, cannot determine theta4 and theta6 uniquely.")
        theta4_ = np.arctan2(-M[1,2]/sin_t5, -M[0,2]/sin_t5)
        theta6_ = np.arctan2(M[2,1]/sin_t5, M[2,0]/sin_t5)
        theta4.append(theta4_)
        theta6.append(theta6_)
    return theta4, theta6

def solve_ik(T_target, d_params, a_params):

    all_solutions = []

    d1, d4, d5, d6 = d_params
    a2, a3 = a_params

    P_c = find_wrist_center(T_target, d6)
    if P_c is None:
        print("No wrist center found; cannot solve IK.")
        return all_solutions
    
    theta1_sols = solve_theta1(P_c, d4)
    if theta1_sols is None:
        print("No valid theta1 solutions; cannot solve IK.")
        return all_solutions

    theta3_sols = solve_theta3(theta1_sols, P_c, d1, a2, a3)
    if theta3_sols is None:
        print("No valid theta3 solutions; cannot solve IK.")
        return all_solutions
    theta2_sols = solve_theta2(theta1_sols, theta3_sols, P_c, d1, a2, a3)
    if theta2_sols is None:
        print("No valid theta2 solutions; cannot solve IK.")
        return all_solutions
    
    for i in range(len(theta1_sols)):
        t1 = theta1_sols[i]
        t2_pos, t2_neg = theta2_sols[i]
        t3_pos, t3_neg = theta3_sols[i]

        arm_solutions_current_t1 = [(t1, t2_pos, t3_pos), (t1, t2_neg, t3_neg)]
        for (t1_sol, t2_sol, t3_sol) in arm_solutions_current_t1:
            T3_6 = solve_wrist_matrix(t1_sol, t2_sol, t3_sol, T_target, d1, a2, a3, d4, d5, d6)
            theta5_sols = solve_theta5(T3_6)
            theta4_sols, theta6_sols = solve_theta4_theta6(T3_6, theta5_sols)

            for j in range(len(theta5_sols)):
                t4 = theta4_sols[j]
                t5 = theta5_sols[j]
                t6 = theta6_sols[j]
                full_solution = (t1_sol, t2_sol, t3_sol, t4, t5, t6)
                all_solutions.append(full_solution)
    return all_solutions

"""Motion Planning"""

def is_point_in_box(point, box_min, box_max):

    px, py, pz = point
    x_min, y_min, z_min = box_min
    x_max, y_max, z_max = box_max

    if (x_min <= px <= x_max) and (y_min <= py <= y_max) and (z_min <= pz <= z_max):
        return True
    else:
        return False
    
def check_collision_pose(q,box):
    # Placeholder function for collision checking
    # In real implementation, this would use robot model and environment
    return False
def check_collision_path(path, box):
    for q in path:
        if check_collision_pose(q, box):
            return True
    return False


def get_random_pose():
    # Trả về 6 góc ngẫu nhiên từ -pi đến pi
    return np.random.uniform(-np.pi, np.pi, 6)

def find_nearest_neighbor(tree, q_rand):
    # Tìm nút (tư thế) trên cây gần q_rand nhất
    best_dist = np.inf
    nearest_node = None
    for node in tree:
        dist = np.linalg.norm(node - q_rand) # Khoảng cách Euclid 6D
        if dist < best_dist:
            best_dist = dist
            nearest_node = node
    return nearest_node

def steer(q_near, q_rand, step_size):
    # Di chuyển một đoạn 'step_size' từ q_near về phía q_rand
    direction_vec = q_rand - q_near
    length = np.linalg.norm(direction_vec)
    
    if length == 0: 
        return q_near # Tránh lỗi chia cho 0
    
    # Chuẩn hóa vector (biến nó thành vector đơn vị, độ dài = 1)
    unit_vec = direction_vec / length
    
    # Tính q_new
    q_new = q_near + unit_vec * step_size
    return q_new

"""Main RRT"""

def plan_rrt(q_start, q_goal, box_obstacle, max_iter, step_size):
    # 1. Bắt đầu cây với tư thế ban đầu
    tree = [q_start] 
    
    print("Starting planning RRT...")
    
    for i in range(max_iter): # 2. Lặp lại
        
        # 2a. Lấy tư thế ngẫu nhiên
        q_rand = get_random_pose()
        
        # 2b. Tìm nút gần nhất
        q_near = find_nearest_neighbor(tree, q_rand)
        
        # 2c. Tạo nút mới (hướng về q_rand)
        q_new = steer(q_near, q_rand, step_size)
        
        # 2d. Kiểm tra va chạm (DÙNG HÀM GIẢ)
        # Chúng ta gọi hàm "giả" check_collision_path
        if not check_collision_path(q_near, q_new, box_obstacle):
            
            # 2e. Thêm vào cây
            tree.append(q_new)
            
            # 3. Kiểm tra xem đã đến gần mục tiêu chưa
            if np.linalg.norm(q_new - q_goal) <= step_size:
                print(f"Found path after {i} iteration!")
                tree.append(q_goal) # Thêm điểm cuối cùng
                return tree # Trả về đường đi
    
    print("Cannot find path with maximum iteration.")
    return None # Không tìm thấy
