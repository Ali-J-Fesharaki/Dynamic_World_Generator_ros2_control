import math
from utils.color_utils import get_color

def export_to_usda(models, output_path):
    """
    Exports a list of static obstacles and walls to a USDA format string 
    and writes it to the output path.
    """
    
    usda_header = '''#usda 1.0
(
    defaultPrim = "World"
    metersPerUnit = 1.0
    upAxis = "Z"
)

def Xform "World"
{
    def PhysicsScene "PhysicsScene"
    {
        vector3f physics:gravityDirection = (0, 0, -1)
        float physics:gravityMagnitude = 9.81
    }

    def Mesh "GroundPlane" (
        prepend apiSchemas = ["PhysicsCollisionAPI"]
    )
    {
        int[] faceVertexCounts = [4]
        int[] faceVertexIndices = [0, 1, 2, 3]
        point3f[] points = [(-50, -50, 0), (50, -50, 0), (50, 50, 0), (-50, 50, 0)]
        color3f[] primvars:displayColor = [(0.5, 0.5, 0.5)]
        double3 xformOp:translate = (0, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
        
        uniform token physics:approximation = "meshSimplification"
        bool physics:collisionEnabled = 1
    }

    def DistantLight "Sun"
    {
        float inputs:angle = 1
        float inputs:intensity = 3000
        color3f inputs:color = (1, 1, 1)
        double3 xformOp:rotateXYZ = (-45, 0, 0)
        uniform token[] xformOpOrder = ["xformOp:rotateXYZ"]
    }

    def Xform "StaticObstacles"
    {
'''
    
    usda_footer = '''    }
}
'''
    
    prims_usd = []
    
    for model in models:
        if model.get("status") == "removed":
            continue
        
        # Skip dynamic obstacles (they will be spawned via ROS2 script)
        if "motion" in model.get("properties", {}):
            continue
            
        m_type = model["type"]
        name = model["name"]
        props = model["properties"]
        color_rgb = get_color(props.get("color", "Gray"))
        
        if m_type == "wall":
            # Wall logic
            start = props["start"]
            end = props["end"]
            width = props["width"]
            height = props["height"]
            
            dx = end[0] - start[0]
            dy = end[1] - start[1]
            length = math.hypot(dx, dy)
            
            cx = start[0] + dx / 2.0
            cy = start[1] + dy / 2.0
            cz = height / 2.0
            
            yaw = math.atan2(dy, dx)
            yaw_deg = math.degrees(yaw)
            
            prim = f'''        def Cube "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            double size = 1.0
            double3 xformOp:translate = ({cx}, {cy}, {cz})
            double3 xformOp:scale = ({length}, {width}, {height})
            double3 xformOp:rotateXYZ = (0, 0, {yaw_deg})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:rotateXYZ", "xformOp:scale"]
            color3f[] primvars:displayColor = [{color_rgb}]
            bool physics:collisionEnabled = 1
        }}
'''
            prims_usd.append(prim)
            
        elif m_type == "box":
            pos = props["position"]
            size = props["size"]
            
            prim = f'''        def Cube "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            double size = 1.0
            double3 xformOp:translate = ({pos[0]}, {pos[1]}, {pos[2]})
            double3 xformOp:scale = ({size[0]}, {size[1]}, {size[2]})
            uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]
            color3f[] primvars:displayColor = [{color_rgb}]
            bool physics:collisionEnabled = 1
        }}
'''
            prims_usd.append(prim)
            
        elif m_type == "cylinder":
            pos = props["position"]
            size = props["size"] # R, H
            
            prim = f'''        def Cylinder "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            double radius = {size[0]}
            double height = {size[1]}
            token axis = "Z"
            double3 xformOp:translate = ({pos[0]}, {pos[1]}, {pos[2]})
            uniform token[] xformOpOrder = ["xformOp:translate"]
            color3f[] primvars:displayColor = [{color_rgb}]
            bool physics:collisionEnabled = 1
        }}
'''
            prims_usd.append(prim)
            
        elif m_type == "sphere":
            pos = props["position"]
            size = props["size"] # R
            
            prim = f'''        def Sphere "{name}" (
            prepend apiSchemas = ["PhysicsCollisionAPI"]
        )
        {{
            double radius = {size[0]}
            double3 xformOp:translate = ({pos[0]}, {pos[1]}, {pos[2]})
            uniform token[] xformOpOrder = ["xformOp:translate"]
            color3f[] primvars:displayColor = [{color_rgb}]
            bool physics:collisionEnabled = 1
        }}
'''
            prims_usd.append(prim)

    content = usda_header + "".join(prims_usd) + usda_footer
    
    with open(output_path, "w") as f:
        f.write(content)
    
    return True
