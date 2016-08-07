b2EdgeShape* EdgeShape_new() {
    return new b2EdgeShape();
}
void EdgeShape_drop(b2EdgeShape* self) {
    delete self;
}

b2Shape* EdgeShape_as_shape(b2EdgeShape* self) {
    return static_cast<b2Shape*>(self);
}
b2EdgeShape* Shape_as_edge_shape(b2Shape* self) {
    return static_cast<b2EdgeShape*>(self);
}

void EdgeShape_set(b2EdgeShape* self,
                   const b2Vec2* v1, const b2Vec2* v2) {
    self->Set(*v1, *v2);
}

b2Vec2 EdgeShape_get_v1(const b2EdgeShape* self) {
    return self->m_vertex1;
}

void EdgeShape_set_v1(b2EdgeShape* self, b2Vec2 v1) {
    self->m_vertex1 = v1;
}

b2Vec2 EdgeShape_get_v2(const b2EdgeShape* self) {
    return self->m_vertex2;
}

void EdgeShape_set_v2(b2EdgeShape* self, b2Vec2 v2) {
    self->m_vertex2 = v2;
}

bool EdgeShape_get_v0(const b2EdgeShape* self, b2Vec2* v0) {
    *v0 = self->m_vertex0;
    return self->m_hasVertex0;
}

void EdgeShape_set_v0(b2EdgeShape* self, const b2Vec2* v0) {
    if (v0) {
        self->m_vertex0 = *v0;
        self->m_hasVertex0 = true;
    } else {
        self->m_hasVertex0 = false;
    }
}

bool EdgeShape_get_v3(const b2EdgeShape* self, b2Vec2* v3) {
    *v3 = self->m_vertex3;
    return self->m_hasVertex3;
}

void EdgeShape_set_v3(b2EdgeShape* self, const b2Vec2* v3) {
    if (v3) {
        self->m_vertex3 = *v3;
        self->m_hasVertex3 = true;
    } else {
        self->m_hasVertex3 = false;
    }
}