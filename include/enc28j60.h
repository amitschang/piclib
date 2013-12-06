/******************************************************************************
* Register locations
******************************************************************************/
// Bank 0 registers --------
#define ERDPTL		0x000
#define ERDPTH		0x001
#define EWRPTL		0x002
#define EWRPTH		0x003
#define ETXSTL		0x004
#define ETXSTH		0x005
#define ETXNDL		0x006
#define ETXNDH		0x007
#define ERXSTL		0x008
#define ERXSTH		0x009
#define ERXNDL		0x00A
#define ERXNDH		0x00B
#define ERXRDPTL	0x00C
#define ERXRDPTH	0x00D
#define ERXWRPTL	0x00E
#define ERXWRPTH	0x00F
#define EDMASTL		0x010
#define EDMASTH		0x011
#define EDMANDL		0x012
#define EDMANDH		0x013
#define EDMADSTL	0x014
#define EDMADSTH	0x015
#define EDMACSL		0x016
#define EDMACSH		0x017
#define EIE		0x01B
#define EIR		0x01C
#define ESTAT		0x01D
#define ECON2		0x01E
#define ECON1		0x01F
// Bank 1 registers -----
#define EHT0		0x100
#define EHT1		0x101
#define EHT2		0x102
#define EHT3		0x103
#define EHT4		0x104
#define EHT5		0x105
#define EHT6		0x106
#define EHT7		0x107
#define EPMM0		0x108
#define EPMM1		0x109
#define EPMM2		0x10A
#define EPMM3		0x10B
#define EPMM4		0x10C
#define EPMM5		0x10D
#define EPMM6		0x10E
#define EPMM7		0x10F
#define EPMCSL		0x110
#define EPMCSH		0x111
#define EPMOL		0x114
#define EPMOH		0x115
#define ERXFCON		0x118
#define EPKTCNT		0x119
// Bank 2 registers -----
#define MACON1		0x200
#define MACON3		0x202
#define MACON4		0x203
#define MABBIPG		0x204
#define MAIPGL		0x206
#define MAIPGH		0x207
#define MACLCON1	0x208
#define MACLCON2	0x209
#define MAMXFLL		0x20A
#define MAMXFLH		0x20B
#define MICMD		0x212
#define MIREGADR	0x214
#define MIWRL		0x216
#define MIWRH		0x217
#define MIRDL		0x218
#define MIRDH		0x219
// Bank 3 registers -----
#define MAADR5		0x300
#define MAADR6		0x301
#define MAADR3		0x302
#define MAADR4		0x303
#define MAADR1		0x304
#define MAADR2		0x305
#define EBSTSD		0x306
#define EBSTCON		0x307
#define EBSTCSL		0x308
#define EBSTCSH		0x309
#define MISTAT		0x30A
#define EREVID		0x312
#define ECOCON		0x315
#define EFLOCON		0x317
#define EPAUSL		0x318
#define EPAUSH		0x319
