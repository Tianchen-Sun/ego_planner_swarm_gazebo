            
from scipy.spatial.transform import Rotation as R
quat_list = [-0.35, 1.23e-06, 4.18e-08, 0.39]
Rm = R.from_quat(quat_list) 

rotation_matrix = Rm.as_matrix()    
print("type_Rm:",type(Rm)) 
print("type_rmatrix:",type(rotation_matrix)) 