typedef void (*DrawPolygonCB)(const b2Vec2*, i32, const b2Color*, RustObject);
typedef void (*DrawCircleCB)(const b2Vec2*, f32, const b2Color*, RustObject);
typedef void (*DrawSolidCircleCB)(const b2Vec2*, f32, const b2Vec2*, const b2Color*, RustObject);
typedef void (*DrawSegmentCB)(const b2Vec2*, const b2Vec2*, const b2Color*, RustObject);
typedef void (*DrawTransformCB)(const b2Transform*, RustObject);

struct CDraw: public b2Draw {
    CDraw() {}
    ~CDraw() override {}
    
    void DrawPolygon(const b2Vec2* vertices, i32 count,
                     const b2Color& color) override {
        draw_polygon(vertices, count, &color, draw_polygon_object);
    }
    
    void DrawSolidPolygon(const b2Vec2* vertices, i32 count,
                          const b2Color& color) override {
        draw_solid_polygon(vertices, count, &color, draw_solid_polygon_object);                  
    }
    
    void DrawCircle(const b2Vec2& center, f32 radius,
                    const b2Color& color) override {
        draw_circle(&center, radius, &color, draw_circle_object);            
    }
    
    void DrawSolidCircle(const b2Vec2& center, f32 radius,
                         const b2Vec2& axis, const b2Color& color) override {
        draw_solid_circle(&center, radius, &axis, &color, draw_solid_circle_object);            
    }
    
    void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override {
        draw_segment(&p1, &p2, &color, draw_segment_object);
    }
    
    void DrawTransform(const b2Transform& xf) override {
        draw_transform(&xf, draw_transform_object);
    }
    
    DrawPolygonCB draw_polygon;
    DrawPolygonCB draw_solid_polygon;
    DrawCircleCB draw_circle;
    DrawSolidCircleCB draw_solid_circle;
    DrawSegmentCB draw_segment;
    DrawTransformCB draw_transform;
    RustObject draw_polygon_object;
    RustObject draw_solid_polygon_object;
    RustObject draw_circle_object;
    RustObject draw_solid_circle_object;
    RustObject draw_segment_object;
    RustObject draw_transform_object;
};

CDraw* CDraw_new(DrawPolygonCB dp,
                 DrawPolygonCB dsp,
                 DrawCircleCB dc,
                 DrawSolidCircleCB dsc,
                 DrawSegmentCB ds,
                 DrawTransformCB dt,
                 RustObject dp_o,
                 RustObject dsp_o,
                 RustObject dc_o,
                 RustObject dsc_o,
                 RustObject ds_o,
                 RustObject dt_o) {
    CDraw* d = new CDraw();
    d->draw_polygon = dp;
    d->draw_solid_polygon = dsp;
    d->draw_circle = dc;
    d->draw_solid_circle = dsc;
    d->draw_segment = ds;
    d->draw_transform = dt;
    d->draw_polygon_object = dp_o;
    d->draw_solid_polygon_object = dsp_o;
    d->draw_circle_object = dc_o;
    d->draw_solid_circle_object = dsc_o;
    d->draw_segment_object = ds_o;
    d->draw_transform_object = dt_o;
    return d;
}

b2Draw* CDraw_as_base(CDraw* self) {
    return static_cast<b2Draw*>(self);
}

void CDraw_drop(CDraw* self) {
    delete self;
}
