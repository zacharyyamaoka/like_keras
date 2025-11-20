

SDF Textures:

    https://gazebosim.org/api/gazebo/3/migrationsdf.html#:~:text=%3Calbedo_map%3E-,texture.png,-%3C/albedo_map%3E
    https://github.com/PX4/PX4-gazebo-models/blob/main/models/arucotag/model.sdf
    In SDF, you can use a plane and apply a material, however URDF doesn't have a plane:

    ```
        <material>
        <pbr>
            <metal>
            <albedo_map>texture.png</albedo_map>
            </metal>
        </pbr>
        </material>
    ```

URDF Texture:
    Inspired by: 
    https://github.com/mikaelarguedas/gazebo_models/tree/master/ar_tags/model/marker0

- Edited it so that the cube can be scaled to real dimensions
- Instead of directly editing the xml, you can just use the same file names