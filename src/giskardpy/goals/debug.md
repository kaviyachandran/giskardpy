position of gripper - 1.284, 0.31482, 0.18757
orientation - 0.8132, 0.048088, -0.57545, -0.072236

## Without casadi
pose - 1.0852, 0.30499, 0.38458
orientation - -0.6811, -0.00033181, 0.7321, -0.00039


## without cas to align

door_P_intermediate_pt :  [[-3.99515331e-01]
 [-3.99515331e-01]
 [ 9.52016244e-17]
 [ 1.00000000e+00]]
door_P_rotated_pt :  [[-2.16216139e-01]
 [-5.21991936e-01]
 [ 9.52016244e-17]
 [ 1.00000000e+00]]
root_P_rotated_pt :  [[1.06878386]
 [0.2       ]
 [0.70699194]
 [1.        ]]

### with points b4 rotation alone
door_P_intermediate_pt :  [[-6.66133815e-16]
 [-5.65000000e-01]
 [-4.63448112e-17]
 [ 1.00000000e+00]]
door_P_rotated_pt :  [[ 2.16216139e-01]
 [-5.21991936e-01]
 [-4.63448112e-17]
 [ 1.00000000e+00]]
[ERROR] [1701197102.477645]: Got a transition callback on a goal handle that we're not tracking
root_P_rotated_pt :  [[1.06878386]
 [0.2       ]
 [0.70699194]
 [1.        ]]


### with pts after rotation alone
door_P_intermediate_pt :  [[-3.99515331e-01]
 [-3.99515331e-01]
 [ 9.52016244e-17]
 [ 1.00000000e+00]]
door_P_rotated_pt :  [[-2.16216139e-01]
 [-5.21991936e-01]
 [ 9.52016244e-17]
 [ 1.00000000e+00]]
root_P_rotated_pt :  [[1.06878386]
 [0.2       ]
 [0.70699194]
 [1.        ]]

root_P_door b4 rot header: 
pose: 
  position: 
    x: 1.285
    y: 0.20000000000000007
    z: 0.185
  orientation: 
    x: -4.3297802811774664e-17
    y: -0.7071067811865475
    z: 0.7071067811865476
    w: 4.3297802811774664e-17
root_P_door after rot header: 

pose: 
  position: 
    x: 1.285
    y: 0.20000000000000007
    z: 0.185
  orientation: 
    x: -0.27059805007309856
    y: -0.6532814824381883
    z: 0.6532814824381884
    w: -0.2705980500730985



### closest pnt on plane

computed point [[1.285 0.305 0.535 1.   ]]
point d_p:  [[-0.24748737]
 [-0.24748737]
 [-0.105     ]
 [ 1.        ]]
rotated point [[ 5.55111512e-17]
 [-3.50000000e-01]
 [-1.05000000e-01]
 [ 1.00000000e+00]]
shapes:  (4, 4) (4, 1)
point_rotated_m_p [1.03751263] [0.305] [0.43248737]

def distance_point_to_rectangular_surface(frame_P_current: Point3, frame_P_bottom_left: Point3,
                                          frame_P_bottom_right: Point3, frame_P_top_left: Point3)-> \
        Tuple[Expression, Point3]: ...