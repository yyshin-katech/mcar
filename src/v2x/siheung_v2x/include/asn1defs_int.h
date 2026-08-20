/*
 * ASN1 definitions (internal) 
 * Copyright (C) 2011-2018 Fabrice Bellard
 */
#ifndef _ASN1_DEFS_INT_H
#define _ASN1_DEFS_INT_H

#include <asn1defs.h>

#if defined(_WIN32)

#include <limits.h>

/* some useful C99 defines which are not defined in MSVC CRT library */
#define snprintf _snprintf
#define vsnprintf _vsnprintf

#ifndef DBL_MAX 
#define DBL_MAX 1.7976931348623157e+308 /* mingw bug: not defined */
#endif

#define INFINITY (DBL_MAX+DBL_MAX)
#define NAN (INFINITY-INFINITY)

static inline int isnan(double x)
{
    return _isnan(x);
}

static inline int isinf(double x)
{
    return !_finite(x) && !_isnan(x);
}

#endif /* !_WIN32 */

#define likely(x)       __builtin_expect(!!(x), 1)
#define unlikely(x)     __builtin_expect(!!(x), 0)

#define force_inline inline __attribute__((always_inline))
#define no_inline __attribute__((noinline))

#define countof(x) (sizeof(x) / sizeof(x[0]))

#define xglue(x, y) x ## y
#define glue(x, y) xglue(x, y)
#define stringify(s)    tostring(s)
#define tostring(s)     #s

typedef struct ASN1ValueStack {
    struct ASN1ValueStack *prev; /* previous entry */
    const ASN1CType *type;
    void *data;
} ASN1ValueStack;

static inline int max_int(int a, int b)
{
    if (a > b)
        return a;
    else
        return b;
}

static inline int min_int(int a, int b)
{
    if (a < b)
        return a;
    else
        return b;
}

static inline int get_bit(const uint8_t *data, int index)
{
    return (data[index >> 3] >> (7 - (index & 7))) & 1;
}

static inline void put_bit(uint8_t *data, int index, int bit)
{
    data[index >> 3] |= bit << (7 - (index & 7));
}

static inline int clz32(unsigned int a)
{
    return __builtin_clz(a);
}

#if defined(__i386__) || defined(__x86_64__)
static inline uint32_t asn1_bswap_32(uint32_t v)
{
    asm ("bswap %k0" : "=r" (v) : "0" (v));
    return v;
}

static inline uint32_t to_be32(const uint8_t *d)
{
    return asn1_bswap_32(*(uint32_t *)d);
}

static inline void from_be32(uint8_t *d, uint32_t v)
{
    *(uint32_t *)d = asn1_bswap_32(v);
}
#else
static inline uint32_t asn1_bswap_32(uint32_t v)
{
    return ((v & 0xff000000) >> 24) | ((v & 0x00ff0000) >>  8) |
        ((v & 0x0000ff00) <<  8) | ((v & 0x000000ff) << 24);
}

static inline uint32_t to_be32(const uint8_t *d)
{
    return (d[0] << 24) | (d[1] << 16) | (d[2] << 8) | d[3];
}

static inline void from_be32(uint8_t *d, uint32_t v)
{
    d[0] = v >> 24;
    d[1] = v >> 16;
    d[2] = v >> 8;
    d[3] = v >> 0;
}
#endif

static inline BOOL check_malloc_size_overflow(size_t *psize,
                                              size_t size1, size_t size2)
{
    if (size1 <= UINT32_MAX && size2 <= UINT32_MAX) {
        /* 32 bit case or fast 64 bit case */
        uint64_t size = (uint64_t)size1 * (uint64_t)size2;
        if (unlikely(size > SIZE_MAX))
            return TRUE;
        *psize = size;
    } else if (size2 == 0) {
        *psize = 0;
    } else {
        if (size1 > (SIZE_MAX / size2))
            return TRUE;
        *psize = size1 * size2;
    }
    return FALSE;
}
    
/* same as asn1_malloc(size1 * size2), but checks overflow of the product. */
static inline void *asn1_malloc2(size_t size1, size_t size2)
{
    size_t size;
    if (check_malloc_size_overflow(&size, size1, size2))
        return NULL;
    return asn1_malloc(size);
}

static inline void *asn1_mallocz2(size_t size1, size_t size2)
{
    size_t size;
    if (check_malloc_size_overflow(&size, size1, size2))
        return NULL;
    return asn1_mallocz(size);
}

/* same as asn1_realloc(ptr, size1 * size2), but checks overflow of
   the product. */
static inline void *asn1_realloc2(void *ptr, size_t size1, size_t size2)
{
    size_t size;
    if (check_malloc_size_overflow(&size, size1, size2))
        return NULL;
    return asn1_realloc(ptr, size);
}

typedef struct ASN1ByteBuffer {
    uint8_t *buf;
    size_t len;
    size_t size;
    BOOL has_error; /* true if a memory allocation occured */
} ASN1ByteBuffer;

void asn1_byte_buffer_init(ASN1ByteBuffer *s);
asn1_exception int __asn1_byte_buffer_realloc(ASN1ByteBuffer *s, size_t size);

static inline asn1_exception int asn1_byte_buffer_expand(ASN1ByteBuffer *s, size_t added_len)
{
    size_t size = s->len + added_len;
    if (unlikely(size > s->size))
        return __asn1_byte_buffer_realloc(s, size);
    return 0;
}

static force_inline void asn1_put_byte(ASN1ByteBuffer *s, int b)
{
    if (unlikely(asn1_byte_buffer_expand(s, 1)))
        return;
    s->buf[s->len++] = b;
}

static force_inline void asn1_put_be16(ASN1ByteBuffer *s, unsigned int b)
{
    if (unlikely(asn1_byte_buffer_expand(s, 2)))
        return;
    s->buf[s->len++] = b >> 8;
    s->buf[s->len++] = b;
}

static force_inline void asn1_put_be32(ASN1ByteBuffer *s, unsigned int b)
{
    if (unlikely(asn1_byte_buffer_expand(s, 4)))
        return;
    s->buf[s->len++] = b >> 24;
    s->buf[s->len++] = b >> 16;
    s->buf[s->len++] = b >> 8;
    s->buf[s->len++] = b;
}

void asn1_put_bytes(ASN1ByteBuffer *s, const uint8_t *buf, size_t count);
void asn1_puts(ASN1ByteBuffer *s, const char *str);
void asn1_vprintf(ASN1ByteBuffer *s, const char *fmt, va_list ap);
void __attribute__((format(printf, 2, 3))) 
    asn1_printf(ASN1ByteBuffer *s, const char *fmt, ...);

asn1_exception int asn1_sequence_set_default_value(void *data, const ASN1SequenceField *f);
asn1_exception int asn1_sequence_set_default_fields(const ASN1SequenceField *f1,
                                                    int nb_fields, void *data,
                                                    const uint8_t *table_present);
int asn1_sequence_check_fields(const ASN1SequenceField *f1, int nb_fields, 
                               const uint8_t *table_present, 
                               char *msg, int msg_size);
#define ASN1_UTF8_MAX_LEN 6
int asn1_to_utf8(uint8_t *buf, unsigned int c);
int asn1_from_utf8(const uint8_t *p, int max_len, const uint8_t **pp);

#define ASN1_DOUBLE_DER_MAX_LEN (1 + 2 + 8)

int asn1_encode_real_der(uint8_t *buf, double d);
int asn1_decode_real_ber(const uint8_t *buf, int len, double *pd);
int asn1_cmp_real(double a1, double a2);

void asn1_pstrcpy(char *buf, size_t buf_size, const char *str);

const ASN1CType *asn1_get_constrained_type(const ASN1ValueStack *vs1,
                                           const ASN1CType *p);

/* Note: the integer must not be a large integer */
static inline BOOL asn1_is_uint32(const ASN1CType *p)
{
    int flags = *p++;
    return (!(flags & ASN1_CTYPE_HAS_EXT) && (flags & ASN1_CTYPE_HAS_LOW) &&
            (int)p[0] >= 0);
}

int asn1_find_enum_index(const ASN1CType *p, int nb_fields, int val, int flags);

extern const char *asn1_char_string_str[ASN1_CSTR_COUNT];

#endif /* _ASN1_DEFS_INT_H */
