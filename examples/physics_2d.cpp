// 2D physics playground: a capsule player, crates, balls, a moving platform,
// a coin trigger, a plank bridge and a wrecking ball on joints, mouse
// dragging and a mouse raycast, all in one phys2d_world.
//   build.bat physics_2d run
//
//   A / D, Space     walk, jump
//   left drag        pull a body around (left click on empty space drops a crate)
//   right click      drop a ball
//   F1               cycle the debug view (off / shapes + contacts / + bounds and grid)
//   R                reset
#define PIX_IMPLEMENTATION
#include "pix.h"

#define LAYER_COIN PHYS_LAYER_4

static void draw_body(const phys2d_world& w, const phys2d_body& b, pix_color color) {
    vec2 p = phys2d_lerp_position(w, b);
    float a = b.shape.type == PHYS2D_AABB ? 0.0f : phys2d_lerp_angle(w, b);
    vec2 x = v2(cosf(a), sinf(a)), y = v2(-sinf(a), cosf(a));
    if (b.shape.type == PHYS2D_CIRCLE) {
        pix_draw_circle(p, b.shape.radius, color);
        pix_draw_line(p, p + x * b.shape.radius, 2, pix_rgba(0, 0, 0, 90));   // shows the spin
    } else if (b.shape.type == PHYS2D_CAPSULE) {
        vec2 h = y * b.shape.half_length;
        float r = b.shape.radius;
        vec2 quad[4] = { p - h - x * r, p - h + x * r, p + h + x * r, p + h - x * r };
        pix_draw_polygon(quad, 4, color);
        pix_draw_circle(p - h, r, color);
        pix_draw_circle(p + h, r, color);
    } else {
        vec2 hx = x * b.shape.half.x, hy = y * b.shape.half.y;
        vec2 quad[4] = { p - hx - hy, p + hx - hy, p + hx + hy, p - hx + hy };
        pix_draw_polygon(quad, 4, color);
    }
}

static idx player, platform, coin;

static void build_level(phys2d_world& w) {
    phys2d_destroy_world(w);
    phys2d_create_world(w);
    // ground, walls, a ramp (a rotated static box) and a ledge
    phys2d_add(w, PHYS2D_STATIC, phys2d_box(1600, 60), v2(0, 330));
    phys2d_add(w, PHYS2D_STATIC, phys2d_box(40, 800), v2(-780, -100));
    phys2d_add(w, PHYS2D_STATIC, phys2d_box(40, 800), v2(780, -100));
    phys2d_body ramp = phys2d_make_body(PHYS2D_STATIC, phys2d_box(400, 30), v2(-420, 245));   // low end flush with the floor
    ramp.angle = -0.35f;     // rises to the right: walk up it
    phys2d_add_body(w, ramp);
    phys2d_add(w, PHYS2D_STATIC, phys2d_aabb(260, 24), v2(450, 60));

    // a kinematic platform that carries whatever stands on it
    platform = phys2d_add(w, PHYS2D_KINEMATIC, phys2d_aabb(180, 20), v2(0, 120));

    // a pyramid of crates
    for (int row = 0; row < 6; row++)
        for (int i = 0; i < 6 - row; i++)
            phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(36, 36), v2(250 + (i - (5 - row) * 0.5f) * 37.0f, 282.0f - row * 36.0f));

    // a rope bridge: planks pinned to each other, the ends pinned to the world
    const int planks = 10;
    vec2 start = v2(-700, -120);
    idx prev = PIX_INVALID;
    for (int i = 0; i < planks; i++) {
        idx plank = phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(40, 10), start + v2(20.0f + 40.0f * i, 0));
        phys2d_add_revolute(w, prev, plank, start + v2(40.0f * i, 0));
        prev = plank;
    }
    phys2d_add_revolute(w, prev, PIX_INVALID, start + v2(40.0f * planks, 0));
    phys2d_add(w, PHYS2D_STATIC, phys2d_aabb(30, 300), start + v2(-15, 160));            // posts
    phys2d_add(w, PHYS2D_STATIC, phys2d_aabb(30, 300), start + v2(40.0f * planks + 15, 160));

    // a wrecking ball on a rope, let go from the side so it swings into the crates
    phys2d_body ball = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(30), v2(-150, -350));
    ball.density = 4.0f;
    idx wrecker = phys2d_add_body(w, ball);
    idx rope = phys2d_add_distance(w, PIX_INVALID, wrecker, v2(250, -350), v2(-150, -350));
    phys2d_joint* r = phys2d_get_joint(w, rope);
    r->enable_spring = true;     // with hertz 0: no spring, just slack...
    r->enable_limit = true;      // ...held taut at its length by the limit
    r->upper = r->length;

    // the player: an upright capsule that never tips over
    phys2d_body pl = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_capsule(16, 64), v2(-700, 250));
    pl.fixed_rotation = true;
    pl.friction = 0.0f;              // we drive its x velocity ourselves
    player = phys2d_add_body(w, pl);

    // a coin: a trigger on its own layer that only the player's layer sees
    phys2d_body c = phys2d_make_body(PHYS2D_STATIC, phys2d_circle(14), v2(450, 10));
    c.trigger = true;
    c.layer = LAYER_COIN;
    c.mask = PHYS_LAYER_0;
    coin = phys2d_add_body(w, c);
}

int main() {
    if (!pix_init("pix - 2D physics", 1280, 720)) return 1;
    phys2d_world w = {};
    build_level(w);
    pix_camera2d cam = {};
    cam.zoom = 0.8f;
    unsigned debug_modes[3] = { 0, PHYS2D_DRAW_DEFAULT, PHYS2D_DRAW_DEFAULT | PHYS2D_DRAW_AABBS | PHYS2D_DRAW_GRID };
    int debug = 1, score = 0, bumps = 0;
    float flash = 0;
    idx drag = PIX_INVALID;

    while (pix_frame()) {
        if (pix_key_pressed(KEY_ESC)) pix_quit();
        if (pix_key_pressed(KEY_F1)) debug = (debug + 1) % 3;
        if (pix_key_pressed('R')) { build_level(w); score = 0; drag = PIX_INVALID; }
        cam.offset = v2(pix_width() * 0.5f, pix_height() * 0.5f);
        vec2 mouse = pix_mouse_world_2d(cam);

        // input goes in before the update; the world steps at a fixed 60 Hz
        phys2d_body* pl = phys2d_get_body(w, player);
        float move = (pix_key_down('D') ? 1.0f : 0.0f) - (pix_key_down('A') ? 1.0f : 0.0f);
        pl->velocity.x = move * 320.0f;
        if (pix_key_pressed(KEY_SPACE) && pl->on_ground) pl->velocity.y = -560.0f;
        if (pix_mouse_pressed(0)) {
            // grab whatever dynamic body is under the cursor, or drop a crate
            idx under[8];
            size_t n = phys2d_overlap_point(w, mouse, under, 8);
            idx grabbed = PIX_INVALID;
            for (size_t i = 0; i < n && i < 8; i++)
                if (w.bodies[under[i]].type == PHYS2D_DYNAMIC && under[i] != player) grabbed = under[i];
            if (grabbed != PIX_INVALID) drag = phys2d_add_mouse(w, grabbed, mouse);
            else phys2d_add(w, PHYS2D_DYNAMIC, phys2d_box(36, 36), mouse);
        }
        if (phys2d_joint* d = phys2d_get_joint(w, drag)) d->target = mouse;
        if (pix_mouse_released(0) && drag != PIX_INVALID) { phys2d_remove_joint(w, drag); drag = PIX_INVALID; }
        if (pix_mouse_pressed(1)) {
            phys2d_body ball = phys2d_make_body(PHYS2D_DYNAMIC, phys2d_circle(16), mouse);
            ball.restitution = 0.7f;
            phys2d_add_body(w, ball);
        }
        phys2d_body* plat = phys2d_get_body(w, platform);     // ping-pong between x = -250 and 150
        if (plat->position.x > 150)  plat->velocity.x = -150;
        if (plat->position.x < -250 || plat->velocity.x == 0) plat->velocity.x = 150;

        phys2d_update(w, pix_dt());

        for (size_t i = 0; i < w.event_count; i++) {
            const phys2d_event& e = w.events[i];
            if (e.type == PHYS2D_TRIGGER_ENTER && e.a == coin) {
                score++;
                phys2d_teleport(w, coin, v2(-500.0f + (score * 337) % 1000, -40.0f - (score * 97) % 200), 0);
            }
            if (e.type == PHYS2D_CONTACT_BEGIN && e.speed > 300) { bumps++; flash = 0.15f; }
        }
        flash = maxf(0, flash - pix_dt());

        // ask the world: what does the player see toward the mouse?
        vec2 eye = pl->position - v2(0, 20);
        phys2d_ray_hit hit;
        bool seen = phys2d_raycast(w, eye, mouse - eye, 2000, &hit, PHYS_LAYER_ALL, player);

        pix_clear(flash > 0 ? pix_rgba(44, 36, 40) : pix_rgba(28, 30, 38));
        pix_begin_2d(cam);
        for (size_t i = 0; i < w.joint_count; i++) {     // ropes and bridge cords under the bodies
            const phys2d_joint& j = w.joints[i];
            if (!j.active || j.type != PHYS2D_JOINT_DISTANCE) continue;
            vec2 a, b;
            phys2d_joint_anchors(w, j, &a, &b);
            pix_draw_line(a, b, 3, pix_rgba(150, 130, 100));
        }
        for (size_t i = 0; i < w.body_count; i++) {
            const phys2d_body& b = w.bodies[i];
            if (!b.active) continue;
            pix_color c = b.trigger ? PIX_YELLOW : i == player ? PIX_SKYBLUE
                        : b.type == PHYS2D_STATIC ? pix_rgba(70, 76, 92)
                        : b.type == PHYS2D_KINEMATIC ? pix_rgba(90, 120, 200)
                        : b.shape.type == PHYS2D_CIRCLE ? pix_rgba(230, 110, 90) : pix_rgba(190, 140, 80);
            draw_body(w, b, c);
        }
        if (seen) {
            pix_draw_line(eye, hit.point, 2, pix_rgba(255, 255, 255, 120));
            pix_draw_line(hit.point, hit.point + hit.normal * 24, 2, PIX_RED);
        }
        if (debug_modes[debug]) pix_draw_physics2d(w, debug_modes[debug], 1.5f);
        pix_end_2d();

        size_t live = 0, asleep = 0;
        for (size_t i = 0; i < w.body_count; i++) { live += w.bodies[i].active; asleep += w.bodies[i].active && w.bodies[i].sleeping; }
        pix_draw_textf(16, 14, 22, PIX_WHITE, "coins %d   hard hits %d   bodies %d (%d asleep)   contacts %d",
                       score, bumps, (int)live, (int)asleep, (int)w.contact_count);
        pix_draw_text("A/D + Space move   LMB drag / crate   RMB ball   F1 debug view   R reset", 16, 44, 18, PIX_LIGHTGRAY);
        pix_draw_fps((float)pix_width() - 90, 12);
    }
    phys2d_destroy_world(w);
    pix_shutdown();
    return 0;
}
