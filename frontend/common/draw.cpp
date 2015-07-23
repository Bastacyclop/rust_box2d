typedef void (*DrawPolygonCB)(RustFatObject, const b2Vec2*, i32, const b2Color*);
typedef void (*DrawCircleCB)(RustFatObject, const b2Vec2*, f32, const b2Color*);
typedef void (*DrawSolidCircleCB)(RustFatObject, const b2Vec2*, f32, const b2Vec2*,
                                  const b2Color*);
typedef void (*DrawSegmentCB)(RustFatObject, const b2Vec2*, const b2Vec2*,
                              const b2Color*);
typedef void (*DrawTransformCB)(RustFatObject, const b2Transform*);

struct DrawLink: public b2Draw {
    DrawLink(): b2Draw() {}
    ~DrawLink() {}

    void DrawPolygon(const b2Vec2* vertices, i32 count,
                     const b2Color& color) {
        draw_polygon(object, vertices, count, &color);
    }

    void DrawSolidPolygon(const b2Vec2* vertices, i32 count,
                          const b2Color& color) {
        draw_solid_polygon(object, vertices, count, &color);
    }

    void DrawCircle(const b2Vec2& center, f32 radius,
                    const b2Color& color) {
        draw_circle(object, &center, radius, &color);
    }

    void DrawSolidCircle(const b2Vec2& center, f32 radius,
                         const b2Vec2& axis, const b2Color& color) {
        draw_solid_circle(object, &center, radius, &axis, &color);
    }

    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2,
                     const b2Color& color) {
        draw_segment(object, &p1, &p2, &color);
    }

    void DrawTransform(const b2Transform& xf) {
        draw_transform(object, &xf);
    }

    RustFatObject object;
    DrawPolygonCB draw_polygon;
    DrawPolygonCB draw_solid_polygon;
    DrawCircleCB draw_circle;
    DrawSolidCircleCB draw_solid_circle;
    DrawSegmentCB draw_segment;
    DrawTransformCB draw_transform;
};

DrawLink* DrawLink_new(RustFatObject o,
                       DrawPolygonCB dp,
                       DrawPolygonCB dsp,
                       DrawCircleCB dc,
                       DrawSolidCircleCB dsc,
                       DrawSegmentCB ds,
                       DrawTransformCB dt) {
    DrawLink* d = new DrawLink();
    d->object = o;
    d->draw_polygon = dp;
    d->draw_solid_polygon = dsp;
    d->draw_circle = dc;
    d->draw_solid_circle = dsc;
    d->draw_segment = ds;
    d->draw_transform = dt;
    return d;
}

b2Draw* DrawLink_as_base(DrawLink* self) {
    return static_cast<b2Draw*>(self);
}

void DrawLink_drop(DrawLink* self) {
    delete self;
}

void DrawLink_set_object(DrawLink* self, RustFatObject o) {
    self->object = o;
}

void DrawLink_set_flags(DrawLink* self, u32 flags) {
    self->SetFlags(flags);
}
