#define RINGBUF_SIZE 512
struct ringbuf {
    uint8_t buf[RINGBUF_SIZE];
    uint32_t begin;
    uint32_t size;
};

static inline void ringbuf_put(struct ringbuf *p, uint8_t b) {
    if (p->size >=  RINGBUF_SIZE) {
        return; // Overrun
    }
    (p->buf)[(p->begin+p->size) % RINGBUF_SIZE] = b;
    ++(p->size);
}

static inline uint8_t ringbuf_get(struct ringbuf *p) {
    uint8_t b = (p->buf)[p->begin];
    p->begin = (p->begin + 1) % RINGBUF_SIZE;
    --(p->size);
    return b;
}

static inline uint32_t ringbuf_size(struct ringbuf *p) {
    return p->size;
}

static inline void ringbuf_init(struct ringbuf *p) {
     p->begin = 0;
     p->size = 0;
}
