b2ChainShape* ChainShape_new() {
    return new b2ChainShape();
}

void ChainShape_drop(b2ChainShape* self) {
    delete self;
}

b2Shape* ChainShape_as_shape(b2ChainShape* self) {
    return static_cast<b2Shape*>(self);
}

b2ChainShape* Shape_as_chain_shape(b2Shape* self) {
    return static_cast<b2ChainShape*>(self);
}

void ChainShape_clear(b2ChainShape* self) {
    self->Clear();
}

void ChainShape_create_loop(b2ChainShape* self,
                            const b2Vec2* vertices,
                            i32 count) {
    self->CreateLoop(vertices, count);
}

void ChainShape_create_chain(b2ChainShape* self,
                             const b2Vec2* vertices,
                             i32 count) {
    self->CreateChain(vertices, count);
}

const b2Vec2* ChainShape_get_vertices_const(const b2ChainShape* self) {
    return self->m_vertices;
}

i32 ChainShape_get_vertex_count(const b2ChainShape* self) {
    return self->m_count;
}

bool ChainShape_get_prev_vertex(const b2ChainShape* self, b2Vec2* prev) {
    *prev = self->m_prevVertex;
    return self->m_hasPrevVertex;
}

void ChainShape_set_prev_vertex(b2ChainShape* self, const b2Vec2* prev) {
    if (prev) {
        self->m_prevVertex = *prev;
        self->m_hasPrevVertex = true;
    } else {
        self->m_hasPrevVertex = false;
    }
}

bool ChainShape_get_next_vertex(const b2ChainShape* self, b2Vec2* next) {
    *next = self->m_nextVertex;
    return self->m_hasNextVertex;
}

void ChainShape_set_next_vertex(b2ChainShape* self, const b2Vec2* next) {
    if (next) {
        self->m_nextVertex = *next;
        self->m_hasNextVertex = true;
    } else {
        self->m_hasNextVertex = false;
    }
}

void ChainShape_get_child_edge(const b2ChainShape* self,
                               b2EdgeShape* edge, i32 index) {
    self->GetChildEdge(edge, index);
}
