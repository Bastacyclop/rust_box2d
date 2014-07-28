typedef void (*DrawPolygonCB)(RustObject, const b2Vec2*, i32, const b2Color*);
typedef void (*DrawCircleCB)(RustObject, const b2Vec2*, f32, const b2Color*);
typedef void (*DrawSolidCircleCB)(RustObject, const b2Vec2*, f32, const b2Vec2*,
                                  const b2Color*);
typedef void (*DrawSegmentCB)(RustObject, const b2Vec2*, const b2Vec2*,
                              const b2Color*);
typedef void (*DrawTransformCB)(RustObject, const b2Transform*);

struct CDraw: public b2Draw {
    CDraw(): b2Draw() {}
    ~CDraw() override {}
    
    void DrawPolygon(const b2Vec2* vertices, i32 count,
                     const b2Color& color) override {
        draw_polygon(object, vertices, count, &color);
    }
    
    void DrawSolidPolygon(const b2Vec2* vertices, i32 count,
                          const b2Color& color) override {
        draw_solid_polygon(object, vertices, count, &color);
    }
    
    void DrawCircle(const b2Vec2& center, f32 radius,
                    const b2Color& color) override {
        draw_circle(object, &center, radius, &color);
    }
    
    void DrawSolidCircle(const b2Vec2& center, f32 radius,
                         const b2Vec2& axis, const b2Color& color) override {
        draw_solid_circle(object, &center, radius, &axis, &color);
    }
    
    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2,
                     const b2Color& color) override {
        draw_segment(object, &p1, &p2, &color);
    }
    
    void DrawTransform(const b2Transform& xf) override {
        draw_transform(object, &xf);
    }
    
    RustObject object;
    DrawPolygonCB draw_polygon;
    DrawPolygonCB draw_solid_polygon;
    DrawCircleCB draw_circle;
    DrawSolidCircleCB draw_solid_circle;
    DrawSegmentCB draw_segment;
    DrawTransformCB draw_transform;
};

CDraw* CDraw_new(RustObject o,
                 DrawPolygonCB dp,
                 DrawPolygonCB dsp,
                 DrawCircleCB dc,
                 DrawSolidCircleCB dsc,
                 DrawSegmentCB ds,
                 DrawTransformCB dt) {
    CDraw* d = new CDraw();
    d->object = o;
    d->draw_polygon = dp;
    d->draw_solid_polygon = dsp;
    d->draw_circle = dc;
    d->draw_solid_circle = dsc;
    d->draw_segment = ds;
    d->draw_transform = dt;
    return d;
}

b2Draw* CDraw_as_base(CDraw* self) {
    return static_cast<b2Draw*>(self);
}

void CDraw_drop(CDraw* self) {
    delete self;
}

void CDraw_set_flags(CDraw* self, u32 flags) {
    self->SetFlags(flags);
}

u32 CDraw_get_flags(const CDraw* self) {
    return self->GetFlags();
}

void CDraw_append_flags(CDraw* self, u32 flags) {
    self->AppendFlags(flags);
}

void CDraw_clear_flags(CDraw* self, u32 flags) {
    self->ClearFlags(flags);
}
