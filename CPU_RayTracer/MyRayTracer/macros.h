#define EPSILON			0.001f

#define MIN(a, b)		( ( a ) < ( b ) ? ( a ) : ( b ) )
#define MAX(a, b)		( ( a ) > ( b ) ? ( a ) : ( b ) )
#define MIN3(a, b, c)		( ( a ) < ( b ) \
? ( ( a ) < ( c ) ? ( a ) : ( c ) ) \
: ( ( b ) < ( c ) ? ( b ) : ( c ) ) )
#define MAX3(a, b, c)		( ( a ) > ( b ) \
? ( ( a ) > ( c ) ? ( a ) : ( c ) ) \
: ( ( b ) > ( c ) ? ( b ) : ( c ) ) )

#define ROUGHNESS       0.0f
#define N_LIGHTS        16
