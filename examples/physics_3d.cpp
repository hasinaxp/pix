// 3D physics playground: stacks, a wrecking ball on a chain, a ragdoll, a
// seesaw, an elevator on a slider, a door on a hinge, a heightfield hill, a
// trigger zone and a walking player, all in one phys3d_world.
//   build.bat physics_3d run
//
//   W A S D, Space     walk, jump            right drag    orbit the camera
//   left drag          pull a body around    E             throw a ball
//   F1                 cycle the debug view  R             reset
#define PIX_IMPLEMENTATION
#include "pix.h"

static idx mesh_cube, mesh_ball, mesh_tube, mesh_ground, mesh_hill;
static idx mat_ground, mat_crate, mat_ball, mat_player, mat_doll, mat_wood, mat_metal, mat_hill, mat_zone;

// one mesh per shape, scaled to the body
static void draw_body(const phys3d_world& w, const phys3d_body& b, idx material, pix_color tint = PIX_WHITE) {
    mat4 t = phys3d_body_transform(w, b);
    const phys3d_shape& s = b.shape;
    if (s.type == PHYS3D_BOX) {
        pix_draw_mesh(mesh_cube, material, mat4_mul(t, mat4_scale(s.half.x * 2, s.half.y * 2, s.half.z * 2)), tint);
    } else if (s.type == PHYS3D_SPHERE) {
        pix_draw_mesh(mesh_ball, material, mat4_mul(t, mat4_scale(s.radius, s.radius, s.radius)), tint);
    } else if (s.type == PHYS3D_CAPSULE) {
        float r = s.radius, h = s.half_length;
        pix_draw_mesh(mesh_tube, material, mat4_mul(t, mat4_scale(r, h * 2, r)), tint);
        pix_draw_mesh(mesh_ball, material, mat4_mul(t, mat4_mul(mat4_translate(0, h, 0), mat4_scale(r, r, r))), tint);
        pix_draw_mesh(mesh_ball, material, mat4_mul(t, mat4_mul(mat4_translate(0, -h, 0), mat4_scale(r, r, r))), tint);
    }
}

static idx player, zone, elevator, elevator_rail;
static idx body_material[PHYS3D_MAX_BODIES];       // what each body is drawn with

// adds a body and remembers how to draw it
static idx add(phys3d_world& w, const phys3d_body& b, idx material) {
    idx id = phys3d_add_body(w, b);
    if (id != PIX_INVALID) body_material[id] = material;
    return id;
}
static idx add(phys3d_world& w, uint8_t type, phys3d_shape shape, vec3 p, idx material) {
    return add(w, phys3d_make_body(type, shape, p), material);
}
static phys3d_mesh hill;
static vec3 hill_origin = v3(-38, 0, -16);

static void add_ragdoll(phys3d_world& w, vec3 o) {
    auto part = [&](phys3d_shape s, vec3 p) { return add(w, PHYS3D_DYNAMIC, s, o + p, mat_doll); };
    idx torso = part(phys3d_box(v3(0.5f, 0.7f, 0.3f)), v3(0, 0, 0));
    idx head  = part(phys3d_sphere(0.15f), v3(0, 0.55f, 0));
    phys3d_get_joint(w, phys3d_add_ball(w, torso, head, o + v3(0, 0.38f, 0)))->swing_limit = 0.6f;
    for (int side = -1; side <= 1; side += 2) {
        float x = 0.37f * side;
        idx upper = part(phys3d_capsule(0.07f, 0.4f), v3(x, 0.1f, 0));
        idx lower = part(phys3d_capsule(0.06f, 0.4f), v3(x, -0.3f, 0));
        phys3d_joint* shoulder = phys3d_get_joint(w, phys3d_add_ball(w, torso, upper, o + v3(x, 0.3f, 0), v3(0, -1, 0)));
        shoulder->swing_limit = 1.4f;
        phys3d_joint* elbow = phys3d_get_joint(w, phys3d_add_hinge(w, upper, lower, o + v3(x, -0.1f, 0), v3(1, 0, 0)));
        elbow->enable_limit = true; elbow->lower = -2.4f; elbow->upper = 0.0f;

        float lx = 0.13f * side;
        idx thigh = part(phys3d_capsule(0.09f, 0.45f), v3(lx, -0.6f, 0));
        idx shin  = part(phys3d_capsule(0.08f, 0.45f), v3(lx, -1.06f, 0));
        phys3d_joint* hip = phys3d_get_joint(w, phys3d_add_ball(w, torso, thigh, o + v3(lx, -0.36f, 0), v3(0, -1, 0)));
        hip->swing_limit = 1.0f;
        phys3d_joint* knee = phys3d_get_joint(w, phys3d_add_hinge(w, thigh, shin, o + v3(lx, -0.83f, 0), v3(1, 0, 0)));
        knee->enable_limit = true; knee->lower = 0.0f; knee->upper = 2.4f;
    }
}

static void build_world(phys3d_world& w) {
    phys3d_destroy_world(w);
    phys3d_create_world(w);
    phys3d_add(w, PHYS3D_STATIC, phys3d_plane(v3(0, 1, 0)), v3(0, 0, 0));
    phys3d_add(w, PHYS3D_STATIC, phys3d_mesh_shape(hill), hill_origin);

    for (int row = 0; row < 6; row++)                                   // a pyramid of crates
        for (int i = 0; i < 6 - row; i++)
            add(w, PHYS3D_DYNAMIC, phys3d_box(v3(1, 1, 1)), v3(8 + (i - (5 - row) * 0.5f) * 1.02f, 0.5f + row, -6), mat_crate);
    for (int i = 0; i < 8; i++)            // and a tower, right where the wrecking ball bottoms out
        add(w, PHYS3D_DYNAMIC, phys3d_box(v3(0.8f, 0.8f, 0.8f)), v3(-2, 0.4f + i * 0.8f, -6), mat_crate);

    // a wrecking ball on a chain of capsules, let go from the side
    vec3 top = v3(-2, 9, -6);
    idx prev = PIX_INVALID;
    for (int i = 0; i < 8; i++) {
        phys3d_body link = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_capsule(0.06f, 0.6f), top + v3(0.5f * i + 0.25f, 0, 0));
        link.rotation = quat_from_axis_angle(v3(0, 0, 1), PIX_PI * 0.5f);
        idx id = add(w, link, mat_metal);
        phys3d_add_ball(w, prev, id, top + v3(0.5f * i, 0, 0));
        prev = id;
    }
    phys3d_body ball = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.7f), top + v3(4.7f, 0, 0));
    ball.density = 3000;
    idx wrecker = add(w, ball, mat_metal);
    phys3d_add_ball(w, prev, wrecker, top + v3(4.0f, 0, 0));

    add_ragdoll(w, v3(2, 5, 3));

    // a seesaw: a plank on a hinge over a block
    add(w, PHYS3D_STATIC, phys3d_box(v3(0.4f, 0.6f, 1)), v3(12, 0.3f, 4), mat_metal);
    idx plank = add(w, PHYS3D_DYNAMIC, phys3d_box(v3(5, 0.15f, 1)), v3(12, 0.68f, 4), mat_wood);
    phys3d_add_hinge(w, PIX_INVALID, plank, v3(12, 0.68f, 4), v3(0, 0, 1));
    add(w, PHYS3D_DYNAMIC, phys3d_box(v3(0.6f, 0.6f, 0.6f)), v3(10.2f, 1.2f, 4), mat_crate);

    // an elevator: a platform on a vertical slider with a motor
    elevator = add(w, PHYS3D_DYNAMIC, phys3d_box(v3(2, 0.2f, 2)), v3(-3, 0.3f, 5), mat_wood);
    elevator_rail = phys3d_add_slider(w, PIX_INVALID, elevator, v3(-3, 0.3f, 5), v3(0, 1, 0));
    phys3d_joint* rail = phys3d_get_joint(w, elevator_rail);
    rail->enable_limit = true; rail->lower = 0.0f; rail->upper = 4.0f;
    rail->enable_motor = true; rail->motor_speed = 1.0f; rail->max_motor_force = 20000;

    // a door on a hinge: walk into it
    phys3d_body door = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_box(v3(1.2f, 2.2f, 0.1f)), v3(0.6f, 1.15f, 9));
    door.density = 300;
    idx door_id = add(w, door, mat_wood);
    phys3d_joint* hinge = phys3d_get_joint(w, phys3d_add_hinge(w, PIX_INVALID, door_id, v3(0, 1.15f, 9), v3(0, 1, 0)));
    hinge->enable_limit = true; hinge->lower = -1.7f; hinge->upper = 1.7f;

    // a glowing zone that knows when the player is in it
    phys3d_body z = phys3d_make_body(PHYS3D_STATIC, phys3d_box(v3(3, 2, 3)), v3(6, 1, 6));
    z.trigger = true;
    zone = add(w, z, mat_zone);

    phys3d_body p = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_capsule(0.35f, 1.8f), v3(0, 0.9f, 2));
    p.fixed_rotation = true;
    p.friction = 0.0f;
    p.never_sleep = true;
    player = add(w, p, mat_player);
}

// a renderable copy of the collision hill, with smooth normals
static idx build_hill_mesh(const phys3d_mesh& m) {
    vertex* v = pix_alloc<vertex>(m.vertex_count);
    for (size_t i = 0; i < m.vertex_count; i++) { v[i].position = m.vertices[i]; v[i].normal = v3(0, 0, 0); v[i].uv = v2(m.vertices[i].x * 0.25f, m.vertices[i].z * 0.25f); }
    for (size_t t = 0; t < m.triangle_count; t++) {
        uint32_t a = m.indices[t * 3], b = m.indices[t * 3 + 1], c = m.indices[t * 3 + 2];
        vec3 n = v3cross(m.vertices[b] - m.vertices[a], m.vertices[c] - m.vertices[a]);
        v[a].normal += n; v[b].normal += n; v[c].normal += n;
    }
    for (size_t i = 0; i < m.vertex_count; i++) v[i].normal = v3norm(v[i].normal);
    idx mesh = load_mesh_data(pix_get_renderer(), v, m.vertex_count, m.indices, m.triangle_count * 3);
    pix_free(v);
    return mesh;
}

int main() {
    if (!pix_init("pix - 3D physics", 1280, 720)) return 1;
    pix_shadows().distance = 50.0f;
    pix_set_time_of_day(pix_get_renderer(), 15.5f);
    mesh_cube = pix_gen_cube(v3(1, 1, 1));
    mesh_ball = pix_gen_sphere(1.0f);
    mesh_tube = pix_gen_cylinder(1.0f, 1.0f);
    mesh_ground = pix_gen_plane(200, 200, 50);
    mat_ground = pix_make_material(pix_rgba(110, 118, 105), 0.0f, 0.95f);
    mat_crate  = pix_make_material(pix_rgba(196, 140, 80), 0.0f, 0.7f);
    mat_ball   = pix_make_material(pix_rgba(210, 70, 60), 0.0f, 0.4f);
    mat_player = pix_make_material(pix_rgba(80, 170, 255), 0.0f, 0.5f);
    mat_doll   = pix_make_material(pix_rgba(235, 220, 200), 0.0f, 0.6f);
    mat_wood   = pix_make_material(pix_rgba(150, 105, 70), 0.0f, 0.8f);
    mat_metal  = pix_make_material(pix_rgba(170, 175, 185), 1.0f, 0.3f);
    mat_hill   = pix_make_material(pix_rgba(95, 140, 80), 0.0f, 0.9f);
    mat_zone   = pix_make_material(pix_rgba(255, 210, 80, 90), 0.0f, 0.5f);

    // a rolling hill to run up, as a heightfield
    const int cols = 33, rows = 33;
    static float heights[cols * rows];
    for (int r = 0; r < rows; r++)
        for (int c = 0; c < cols; c++) {
            float x = (c - 16) / 16.0f, z = (r - 16) / 16.0f;
            heights[r * cols + c] = 4.0f * expf(-2.5f * (x * x + z * z)) + 0.4f * sinf(c * 0.7f) * cosf(r * 0.5f) + 0.3f;
        }
    phys3d_create_heightfield(hill, heights, cols, rows, 1.0f);
    mesh_hill = build_hill_mesh(hill);

    phys3d_world w = {};
    build_world(w);
    float yaw = -0.6f, pitch = 0.45f, dist = 14.0f;
    unsigned debug_modes[3] = { 0, PHYS3D_DRAW_DEFAULT, PHYS3D_DRAW_DEFAULT | PHYS3D_DRAW_AABBS | PHYS3D_DRAW_MESHES };
    int debug = 0;
    bool in_zone = false;
    idx drag = PIX_INVALID;
    float drag_distance = 0;
    camera cam = {};

    while (pix_frame()) {
        float dt = pix_dt();
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed(KEY_F1)) debug = (debug + 1) % 3;
        if (pix_key_pressed('R')) { build_world(w); drag = PIX_INVALID; }
        if (pix_mouse_down(PIX_MOUSE_RIGHT)) {
            vec2 d = pix_mouse_delta();
            yaw += d.x * 0.006f;
            pitch = clampf(pitch + d.y * 0.006f, 0.05f, 1.4f);
        }
        dist = clampf(dist - pix_mouse_wheel() * 1.2f, 4.0f, 40.0f);

        // the player walks relative to the camera
        phys3d_body* pl = phys3d_get_body(w, player);
        vec3 fwd = v3(-cosf(yaw), 0, -sinf(yaw)), right = v3(sinf(yaw), 0, -cosf(yaw));
        vec3 move = v3(0, 0, 0);
        if (pix_key_down('W')) move += fwd;
        if (pix_key_down('S')) move -= fwd;
        if (pix_key_down('D')) move -= right;
        if (pix_key_down('A')) move += right;
        if (v3len(move) > 0) move = v3norm(move) * 5.0f;
        pl->velocity.x = move.x;
        pl->velocity.z = move.z;
        if (pix_key_pressed(KEY_SPACE) && pl->on_ground) pl->velocity.y = 6.0f;

        // drag with the mouse: a ray picks the body, a mouse joint pulls it
        vec3 ray = pix_screen_to_ray(pix_get_renderer(), pix_mouse_position().x, pix_mouse_position().y);
        if (pix_mouse_pressed(PIX_MOUSE_LEFT)) {
            phys3d_ray_hit hit;
            if (phys3d_raycast(w, cam.position, ray, 100, &hit, PHYS_LAYER_ALL, player) && w.bodies[hit.body].type == PHYS3D_DYNAMIC) {
                drag = phys3d_add_mouse(w, hit.body, hit.point);
                drag_distance = hit.distance;
            }
        }
        if (phys3d_joint* d = phys3d_get_joint(w, drag)) d->target = cam.position + ray * drag_distance;
        if (pix_mouse_released(PIX_MOUSE_LEFT) && drag != PIX_INVALID) { phys3d_remove_joint(w, drag); drag = PIX_INVALID; }
        if (pix_key_pressed('E')) {
            phys3d_body shot = phys3d_make_body(PHYS3D_DYNAMIC, phys3d_sphere(0.25f), cam.position + ray * 1.5f);
            shot.velocity = ray * 25.0f;
            shot.density = 2000;
            add(w, shot, mat_ball);
        }
        // the elevator turns round at each end of its rail
        phys3d_joint* rail = phys3d_get_joint(w, elevator_rail);
        float travel = phys3d_get_body(w, elevator)->position.y - 0.3f;
        if (travel > 3.9f) rail->motor_speed = -1.0f;
        if (travel < 0.1f) rail->motor_speed = 1.0f;
        phys3d_wake(w, elevator);

        phys3d_update(w, dt);

        for (size_t i = 0; i < w.event_count; i++) {
            const phys3d_event& e = w.events[i];
            if (e.a == zone && e.b == player) in_zone = e.type == PHYS3D_TRIGGER_ENTER;
        }

        vec3 focus = phys3d_lerp_position(w, *pl) + v3(0, 0.8f, 0);
        cam.position = focus + v3(cosf(yaw) * cosf(pitch), sinf(pitch), sinf(yaw) * cosf(pitch)) * dist;
        cam.direction = focus - cam.position;

        pix_begin_3d(cam);
        pix_draw_mesh(mesh_ground, mat_ground, mat4_identity());
        pix_draw_mesh(mesh_hill, mat_hill, mat4_translate(hill_origin.x, hill_origin.y, hill_origin.z));
        for (size_t i = 0; i < w.body_count; i++) {
            const phys3d_body& b = w.bodies[i];
            if (!b.active || b.shape.type == PHYS3D_PLANE || b.shape.type == PHYS3D_MESH) continue;
            if (b.trigger && !in_zone && debug == 0) continue;     // the zone shows while you stand in it
            draw_body(w, b, body_material[i]);
        }
        if (debug_modes[debug]) pix_draw_physics3d(w, debug_modes[debug]);
        pix_end_3d();

        int asleep = 0;
        for (size_t i = 0; i < w.body_count; i++) asleep += w.bodies[i].active && w.bodies[i].sleeping;
        pix_draw_rect_rounded({ 12, 12, 640, 66 }, 10, pix_rgba(0, 0, 0, 140));
        pix_draw_textf(24, 18, 22, PIX_WHITE, "bodies %d (%d asleep)   contacts %d   joints %d%s", (int)w.body_count, asleep,
                       (int)w.contact_count, (int)w.joint_count, in_zone ? "   - in the zone!" : "");
        pix_draw_text("WASD Space move   right drag orbit   left drag pull   E throw   F1 debug   R reset", 24, 48, 17, PIX_LIGHTGRAY);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    phys3d_destroy_world(w);
    phys3d_destroy_mesh(hill);
    pix_shutdown();
    return 0;
}
