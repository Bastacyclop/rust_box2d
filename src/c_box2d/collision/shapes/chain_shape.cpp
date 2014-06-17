b2ChainShape ChainShape_new() {
    return b2ChainShape();
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
void ChainShape_set_prev_vertex(b2ChainShape* self, const b2Vec2* vertex) {
    self->SetPrevVertex(*vertex);
}
void ChainShape_set_next_vertex(b2ChainShape* self, const b2Vec2* vertex) {
    self->SetNextVertex(*vertex);
}
void ChainShape_get_child_edge(const b2ChainShape* self,
                               b2EdgeShape* edge, i32 index) {
    self->GetChildEdge(edge, index);
}
