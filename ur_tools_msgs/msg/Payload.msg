# Payload Mass in kilograms:
float32 mass

# Payload Center of Gravity, a vector with 
# three elements [CoGx, CoGy, CoGz] specifying 
# the offset (in meters) from the tool mount:
float32 cog_x
float32 cog_y
float32 cog_z

# Payload Inertia Matrix (in kg*m^2), as a vector 
# with six elements [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
# with origin in the CoG and the axes aligned with 
# the tool flange axes:
float32 ixx
float32 iyy
float32 izz
float32 ixy
float32 ixz
float32 iyz