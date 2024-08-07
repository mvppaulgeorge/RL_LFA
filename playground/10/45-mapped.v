// Benchmark "adder" written by ABC on Wed Jul 17 17:29:31 2024

module adder ( 
    \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] , \a[16] ,
    \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] , \a[23] ,
    \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] , \a[30] ,
    \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] , \a[9] ,
    \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] , \b[16] ,
    \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] , \b[23] ,
    \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] , \b[30] ,
    \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ,
    \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] , \s[16] ,
    \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] , \s[23] ,
    \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] , \s[30] ,
    \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] , \s[9]   );
  input  \a[0] , \a[10] , \a[11] , \a[12] , \a[13] , \a[14] , \a[15] ,
    \a[16] , \a[17] , \a[18] , \a[19] , \a[1] , \a[20] , \a[21] , \a[22] ,
    \a[23] , \a[24] , \a[25] , \a[26] , \a[27] , \a[28] , \a[29] , \a[2] ,
    \a[30] , \a[31] , \a[3] , \a[4] , \a[5] , \a[6] , \a[7] , \a[8] ,
    \a[9] , \b[0] , \b[10] , \b[11] , \b[12] , \b[13] , \b[14] , \b[15] ,
    \b[16] , \b[17] , \b[18] , \b[19] , \b[1] , \b[20] , \b[21] , \b[22] ,
    \b[23] , \b[24] , \b[25] , \b[26] , \b[27] , \b[28] , \b[29] , \b[2] ,
    \b[30] , \b[3] , \b[4] , \b[5] , \b[6] , \b[7] , \b[8] , \b[9] ;
  output \s[0] , \s[10] , \s[11] , \s[12] , \s[13] , \s[14] , \s[15] ,
    \s[16] , \s[17] , \s[18] , \s[19] , \s[1] , \s[20] , \s[21] , \s[22] ,
    \s[23] , \s[24] , \s[25] , \s[26] , \s[27] , \s[28] , \s[29] , \s[2] ,
    \s[30] , \s[31] , \s[3] , \s[4] , \s[5] , \s[6] , \s[7] , \s[8] ,
    \s[9] ;
  wire new_n97, new_n98, new_n99, new_n100, new_n101, new_n102, new_n103,
    new_n104, new_n105, new_n106, new_n107, new_n108, new_n109, new_n110,
    new_n111, new_n112, new_n113, new_n114, new_n115, new_n116, new_n117,
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n124,
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n168, new_n169, new_n170,
    new_n171, new_n172, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n351,
    new_n353, new_n354, new_n356, new_n357, new_n358, new_n361, new_n363,
    new_n364, new_n365, new_n367;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1d18x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  nor042aa1n06x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n24x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand22aa1n09x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  norb03aa1d15x5               g006(.a(new_n100), .b(new_n99), .c(new_n101), .out0(new_n102));
  oaih12aa1n06x5               g007(.a(new_n100), .b(\b[3] ), .c(\a[4] ), .o1(new_n103));
  nand42aa1n08x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand02aa1d16x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1d32x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb03aa1d24x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  nor002aa1n03x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  aoi012aa1n12x5               g013(.a(new_n108), .b(new_n106), .c(new_n105), .o1(new_n109));
  oai013aa1n06x5               g014(.a(new_n109), .b(new_n102), .c(new_n107), .d(new_n103), .o1(new_n110));
  xnrc02aa1n12x5               g015(.a(\b[6] ), .b(\a[7] ), .out0(new_n111));
  nor022aa1n16x5               g016(.a(\b[5] ), .b(\a[6] ), .o1(new_n112));
  nand42aa1n06x5               g017(.a(\b[5] ), .b(\a[6] ), .o1(new_n113));
  nanb02aa1n12x5               g018(.a(new_n112), .b(new_n113), .out0(new_n114));
  inv000aa1n02x5               g019(.a(new_n114), .o1(new_n115));
  norp02aa1n12x5               g020(.a(\b[4] ), .b(\a[5] ), .o1(new_n116));
  nand42aa1n06x5               g021(.a(\b[4] ), .b(\a[5] ), .o1(new_n117));
  norb02aa1n03x5               g022(.a(new_n117), .b(new_n116), .out0(new_n118));
  xorc02aa1n12x5               g023(.a(\a[8] ), .b(\b[7] ), .out0(new_n119));
  nano32aa1n03x7               g024(.a(new_n111), .b(new_n119), .c(new_n115), .d(new_n118), .out0(new_n120));
  aoi022aa1d24x5               g025(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  tech160nm_fioai012aa1n05x5   g026(.a(new_n121), .b(new_n116), .c(new_n112), .o1(new_n122));
  oa0022aa1n06x5               g027(.a(\a[8] ), .b(\b[7] ), .c(\a[7] ), .d(\b[6] ), .o(new_n123));
  aoi022aa1n12x5               g028(.a(new_n122), .b(new_n123), .c(\b[7] ), .d(\a[8] ), .o1(new_n124));
  xorc02aa1n12x5               g029(.a(\a[9] ), .b(\b[8] ), .out0(new_n125));
  aoai13aa1n02x5               g030(.a(new_n125), .b(new_n124), .c(new_n120), .d(new_n110), .o1(new_n126));
  nor042aa1n04x5               g031(.a(\b[9] ), .b(\a[10] ), .o1(new_n127));
  nand02aa1d08x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  norb02aa1n06x5               g033(.a(new_n128), .b(new_n127), .out0(new_n129));
  xnbna2aa1n03x5               g034(.a(new_n129), .b(new_n126), .c(new_n98), .out0(\s[10] ));
  inv030aa1n06x5               g035(.a(new_n102), .o1(new_n131));
  nona22aa1n09x5               g036(.a(new_n131), .b(new_n107), .c(new_n103), .out0(new_n132));
  nona23aa1n09x5               g037(.a(new_n119), .b(new_n118), .c(new_n111), .d(new_n114), .out0(new_n133));
  inv040aa1n06x5               g038(.a(new_n124), .o1(new_n134));
  aoai13aa1n12x5               g039(.a(new_n134), .b(new_n133), .c(new_n132), .d(new_n109), .o1(new_n135));
  aoai13aa1n06x5               g040(.a(new_n129), .b(new_n97), .c(new_n135), .d(new_n125), .o1(new_n136));
  oaoi03aa1n02x5               g041(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n137));
  inv000aa1d42x5               g042(.a(new_n137), .o1(new_n138));
  nand42aa1n16x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor042aa1n06x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xobna2aa1n03x5               g046(.a(new_n141), .b(new_n136), .c(new_n138), .out0(\s[11] ));
  oai022aa1n02x5               g047(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n143));
  aoi012aa1n02x5               g048(.a(new_n141), .b(new_n128), .c(new_n143), .o1(new_n144));
  aoi022aa1n02x5               g049(.a(new_n136), .b(new_n144), .c(\b[10] ), .d(\a[11] ), .o1(new_n145));
  nor042aa1n06x5               g050(.a(\b[11] ), .b(\a[12] ), .o1(new_n146));
  nand42aa1n16x5               g051(.a(\b[11] ), .b(\a[12] ), .o1(new_n147));
  norb02aa1n02x5               g052(.a(new_n147), .b(new_n146), .out0(new_n148));
  oaib12aa1n02x5               g053(.a(new_n139), .b(new_n146), .c(new_n147), .out0(new_n149));
  tech160nm_fiao0012aa1n02p5x5 g054(.a(new_n149), .b(new_n136), .c(new_n144), .o(new_n150));
  oaib12aa1n03x5               g055(.a(new_n150), .b(new_n145), .c(new_n148), .out0(\s[12] ));
  nano23aa1n06x5               g056(.a(new_n146), .b(new_n140), .c(new_n147), .d(new_n139), .out0(new_n152));
  nand23aa1d12x5               g057(.a(new_n152), .b(new_n125), .c(new_n129), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n03x5               g059(.a(new_n154), .b(new_n124), .c(new_n120), .d(new_n110), .o1(new_n155));
  norb03aa1n03x5               g060(.a(new_n147), .b(new_n140), .c(new_n146), .out0(new_n156));
  nanp02aa1n04x5               g061(.a(new_n139), .b(new_n128), .o1(new_n157));
  oab012aa1n06x5               g062(.a(new_n157), .b(new_n97), .c(new_n127), .out0(new_n158));
  aoi012aa1n06x5               g063(.a(new_n146), .b(new_n140), .c(new_n147), .o1(new_n159));
  aob012aa1n02x5               g064(.a(new_n159), .b(new_n158), .c(new_n156), .out0(new_n160));
  nanb02aa1n03x5               g065(.a(new_n160), .b(new_n155), .out0(new_n161));
  nanp02aa1n09x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  nor042aa1n06x5               g067(.a(\b[12] ), .b(\a[13] ), .o1(new_n163));
  norb02aa1n02x5               g068(.a(new_n162), .b(new_n163), .out0(new_n164));
  inv040aa1n03x5               g069(.a(new_n159), .o1(new_n165));
  aoi112aa1n02x5               g070(.a(new_n165), .b(new_n164), .c(new_n158), .d(new_n156), .o1(new_n166));
  aoi022aa1n02x5               g071(.a(new_n161), .b(new_n164), .c(new_n155), .d(new_n166), .o1(\s[13] ));
  inv000aa1n06x5               g072(.a(new_n163), .o1(new_n168));
  aoai13aa1n03x5               g073(.a(new_n164), .b(new_n160), .c(new_n135), .d(new_n154), .o1(new_n169));
  nor042aa1n03x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  tech160nm_finand02aa1n05x5   g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n171), .b(new_n170), .out0(new_n172));
  xnbna2aa1n03x5               g077(.a(new_n172), .b(new_n169), .c(new_n168), .out0(\s[14] ));
  nano23aa1n06x5               g078(.a(new_n170), .b(new_n163), .c(new_n171), .d(new_n162), .out0(new_n174));
  aoai13aa1n06x5               g079(.a(new_n174), .b(new_n160), .c(new_n135), .d(new_n154), .o1(new_n175));
  tech160nm_fioaoi03aa1n03p5x5 g080(.a(\a[14] ), .b(\b[13] ), .c(new_n168), .o1(new_n176));
  inv030aa1n02x5               g081(.a(new_n176), .o1(new_n177));
  xorc02aa1n12x5               g082(.a(\a[15] ), .b(\b[14] ), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n175), .c(new_n177), .out0(\s[15] ));
  aoai13aa1n02x5               g084(.a(new_n178), .b(new_n176), .c(new_n161), .d(new_n174), .o1(new_n180));
  tech160nm_fixorc02aa1n03p5x5 g085(.a(\a[16] ), .b(\b[15] ), .out0(new_n181));
  nor042aa1n09x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  norp02aa1n02x5               g087(.a(new_n181), .b(new_n182), .o1(new_n183));
  inv000aa1d42x5               g088(.a(new_n182), .o1(new_n184));
  inv000aa1d42x5               g089(.a(new_n178), .o1(new_n185));
  aoai13aa1n03x5               g090(.a(new_n184), .b(new_n185), .c(new_n175), .d(new_n177), .o1(new_n186));
  aoi022aa1n02x7               g091(.a(new_n186), .b(new_n181), .c(new_n180), .d(new_n183), .o1(\s[16] ));
  nor042aa1d18x5               g092(.a(\b[16] ), .b(\a[17] ), .o1(new_n188));
  nand42aa1d28x5               g093(.a(\b[16] ), .b(\a[17] ), .o1(new_n189));
  nano32aa1d12x5               g094(.a(new_n153), .b(new_n181), .c(new_n174), .d(new_n178), .out0(new_n190));
  aoai13aa1n06x5               g095(.a(new_n190), .b(new_n124), .c(new_n120), .d(new_n110), .o1(new_n191));
  nanp02aa1n02x5               g096(.a(new_n181), .b(new_n178), .o1(new_n192));
  aoai13aa1n06x5               g097(.a(new_n174), .b(new_n165), .c(new_n158), .d(new_n156), .o1(new_n193));
  aoi012aa1n02x7               g098(.a(new_n192), .b(new_n193), .c(new_n177), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\a[16] ), .o1(new_n195));
  aob012aa1n02x5               g100(.a(new_n182), .b(\b[15] ), .c(\a[16] ), .out0(new_n196));
  oaib12aa1n02x5               g101(.a(new_n196), .b(\b[15] ), .c(new_n195), .out0(new_n197));
  nona22aa1n09x5               g102(.a(new_n191), .b(new_n194), .c(new_n197), .out0(new_n198));
  oaib12aa1n02x5               g103(.a(new_n198), .b(new_n188), .c(new_n189), .out0(new_n199));
  oab012aa1n02x4               g104(.a(new_n188), .b(\a[16] ), .c(\b[15] ), .out0(new_n200));
  nanp03aa1n02x5               g105(.a(new_n200), .b(new_n196), .c(new_n189), .o1(new_n201));
  nona22aa1n03x5               g106(.a(new_n191), .b(new_n194), .c(new_n201), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n199), .b(new_n202), .o1(\s[17] ));
  nor042aa1d18x5               g108(.a(\b[17] ), .b(\a[18] ), .o1(new_n204));
  nand42aa1n20x5               g109(.a(\b[17] ), .b(\a[18] ), .o1(new_n205));
  norb02aa1n02x5               g110(.a(new_n205), .b(new_n204), .out0(new_n206));
  xobna2aa1n03x5               g111(.a(new_n206), .b(new_n202), .c(new_n189), .out0(\s[18] ));
  inv000aa1n02x5               g112(.a(new_n197), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n208), .b(new_n192), .c(new_n193), .d(new_n177), .o1(new_n209));
  nano23aa1d15x5               g114(.a(new_n188), .b(new_n204), .c(new_n205), .d(new_n189), .out0(new_n210));
  aoai13aa1n06x5               g115(.a(new_n210), .b(new_n209), .c(new_n135), .d(new_n190), .o1(new_n211));
  oa0012aa1n02x5               g116(.a(new_n205), .b(new_n204), .c(new_n188), .o(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  nand42aa1n20x5               g118(.a(\b[18] ), .b(\a[19] ), .o1(new_n214));
  nor002aa1d32x5               g119(.a(\b[18] ), .b(\a[19] ), .o1(new_n215));
  norb02aa1d21x5               g120(.a(new_n214), .b(new_n215), .out0(new_n216));
  xnbna2aa1n03x5               g121(.a(new_n216), .b(new_n211), .c(new_n213), .out0(\s[19] ));
  xnrc02aa1n02x5               g122(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n06x5               g123(.a(new_n216), .b(new_n212), .c(new_n198), .d(new_n210), .o1(new_n219));
  nor002aa1d32x5               g124(.a(\b[19] ), .b(\a[20] ), .o1(new_n220));
  nand02aa1d24x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  norb02aa1n09x5               g126(.a(new_n221), .b(new_n220), .out0(new_n222));
  inv040aa1n04x5               g127(.a(new_n220), .o1(new_n223));
  aoi012aa1n02x5               g128(.a(new_n215), .b(new_n223), .c(new_n221), .o1(new_n224));
  inv000aa1d42x5               g129(.a(new_n215), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n216), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n225), .b(new_n226), .c(new_n211), .d(new_n213), .o1(new_n227));
  aoi022aa1n03x5               g132(.a(new_n227), .b(new_n222), .c(new_n219), .d(new_n224), .o1(\s[20] ));
  nand23aa1n06x5               g133(.a(new_n210), .b(new_n216), .c(new_n222), .o1(new_n229));
  inv000aa1d42x5               g134(.a(new_n229), .o1(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n209), .c(new_n135), .d(new_n190), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n223), .b(new_n221), .c(\b[18] ), .d(\a[19] ), .o1(new_n232));
  oai112aa1n06x5               g137(.a(new_n205), .b(new_n214), .c(new_n204), .d(new_n188), .o1(new_n233));
  aoi012aa1d18x5               g138(.a(new_n220), .b(new_n215), .c(new_n221), .o1(new_n234));
  oai012aa1d24x5               g139(.a(new_n234), .b(new_n233), .c(new_n232), .o1(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n231), .b(new_n236), .o1(new_n237));
  nand02aa1n08x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  nor042aa1d18x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  norb02aa1n15x5               g144(.a(new_n238), .b(new_n239), .out0(new_n240));
  norb03aa1n06x5               g145(.a(new_n221), .b(new_n215), .c(new_n220), .out0(new_n241));
  nanp02aa1n02x5               g146(.a(new_n214), .b(new_n205), .o1(new_n242));
  oab012aa1n03x5               g147(.a(new_n242), .b(new_n188), .c(new_n204), .out0(new_n243));
  inv000aa1n04x5               g148(.a(new_n234), .o1(new_n244));
  aoi112aa1n02x5               g149(.a(new_n244), .b(new_n240), .c(new_n243), .d(new_n241), .o1(new_n245));
  aoi022aa1n02x5               g150(.a(new_n237), .b(new_n240), .c(new_n231), .d(new_n245), .o1(\s[21] ));
  aoai13aa1n03x5               g151(.a(new_n240), .b(new_n235), .c(new_n198), .d(new_n230), .o1(new_n247));
  nor042aa1n09x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nand02aa1n16x5               g153(.a(\b[21] ), .b(\a[22] ), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(new_n250));
  norp02aa1n02x5               g155(.a(new_n250), .b(new_n239), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n239), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n240), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n252), .b(new_n253), .c(new_n231), .d(new_n236), .o1(new_n254));
  aoi022aa1n03x5               g159(.a(new_n254), .b(new_n250), .c(new_n247), .d(new_n251), .o1(\s[22] ));
  nano22aa1n03x7               g160(.a(new_n229), .b(new_n240), .c(new_n250), .out0(new_n256));
  aoai13aa1n06x5               g161(.a(new_n256), .b(new_n209), .c(new_n135), .d(new_n190), .o1(new_n257));
  nano23aa1d15x5               g162(.a(new_n248), .b(new_n239), .c(new_n249), .d(new_n238), .out0(new_n258));
  aoi012aa1n12x5               g163(.a(new_n248), .b(new_n239), .c(new_n249), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  aoi012aa1n02x5               g165(.a(new_n260), .b(new_n235), .c(new_n258), .o1(new_n261));
  nanp02aa1n03x5               g166(.a(new_n257), .b(new_n261), .o1(new_n262));
  nor002aa1n03x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  and002aa1n12x5               g168(.a(\b[22] ), .b(\a[23] ), .o(new_n264));
  nor042aa1n06x5               g169(.a(new_n264), .b(new_n263), .o1(new_n265));
  aoi112aa1n02x5               g170(.a(new_n265), .b(new_n260), .c(new_n235), .d(new_n258), .o1(new_n266));
  aoi022aa1n02x5               g171(.a(new_n262), .b(new_n265), .c(new_n257), .d(new_n266), .o1(\s[23] ));
  aoai13aa1n06x5               g172(.a(new_n258), .b(new_n244), .c(new_n243), .d(new_n241), .o1(new_n268));
  aoi112aa1n02x5               g173(.a(new_n248), .b(new_n263), .c(new_n239), .d(new_n249), .o1(new_n269));
  nanp02aa1n02x5               g174(.a(new_n268), .b(new_n269), .o1(new_n270));
  inv040aa1n02x5               g175(.a(new_n270), .o1(new_n271));
  xorc02aa1n12x5               g176(.a(\a[24] ), .b(\b[23] ), .out0(new_n272));
  aoai13aa1n03x5               g177(.a(new_n272), .b(new_n264), .c(new_n257), .d(new_n271), .o1(new_n273));
  orn002aa1n02x5               g178(.a(new_n272), .b(new_n264), .o(new_n274));
  aoai13aa1n03x5               g179(.a(new_n273), .b(new_n274), .c(new_n271), .d(new_n257), .o1(\s[24] ));
  nano32aa1n02x5               g180(.a(new_n229), .b(new_n272), .c(new_n258), .d(new_n265), .out0(new_n276));
  aoai13aa1n06x5               g181(.a(new_n276), .b(new_n209), .c(new_n135), .d(new_n190), .o1(new_n277));
  and002aa1n02x7               g182(.a(new_n272), .b(new_n265), .o(new_n278));
  inv040aa1n03x5               g183(.a(new_n278), .o1(new_n279));
  orn002aa1n02x5               g184(.a(\a[23] ), .b(\b[22] ), .o(new_n280));
  oao003aa1n02x5               g185(.a(\a[24] ), .b(\b[23] ), .c(new_n280), .carry(new_n281));
  aoai13aa1n12x5               g186(.a(new_n281), .b(new_n279), .c(new_n268), .d(new_n259), .o1(new_n282));
  inv000aa1n02x5               g187(.a(new_n282), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n277), .b(new_n283), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoai13aa1n06x5               g190(.a(new_n278), .b(new_n260), .c(new_n235), .d(new_n258), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n285), .o1(new_n287));
  and003aa1n02x5               g192(.a(new_n286), .b(new_n287), .c(new_n281), .o(new_n288));
  aoi022aa1n02x5               g193(.a(new_n284), .b(new_n285), .c(new_n277), .d(new_n288), .o1(\s[25] ));
  aoai13aa1n03x5               g194(.a(new_n285), .b(new_n282), .c(new_n198), .d(new_n276), .o1(new_n290));
  xorc02aa1n03x5               g195(.a(\a[26] ), .b(\b[25] ), .out0(new_n291));
  nor042aa1n09x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  norp02aa1n02x5               g197(.a(new_n291), .b(new_n292), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n292), .o1(new_n294));
  aoai13aa1n04x5               g199(.a(new_n294), .b(new_n287), .c(new_n277), .d(new_n283), .o1(new_n295));
  aoi022aa1n03x5               g200(.a(new_n295), .b(new_n291), .c(new_n290), .d(new_n293), .o1(\s[26] ));
  and002aa1n12x5               g201(.a(new_n291), .b(new_n285), .o(new_n297));
  nano23aa1n06x5               g202(.a(new_n279), .b(new_n229), .c(new_n297), .d(new_n258), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n209), .c(new_n135), .d(new_n190), .o1(new_n299));
  nor042aa1n04x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  and002aa1n12x5               g205(.a(\b[26] ), .b(\a[27] ), .o(new_n301));
  norp02aa1n06x5               g206(.a(new_n301), .b(new_n300), .o1(new_n302));
  oao003aa1n06x5               g207(.a(\a[26] ), .b(\b[25] ), .c(new_n294), .carry(new_n303));
  inv000aa1d42x5               g208(.a(new_n303), .o1(new_n304));
  aoi112aa1n02x5               g209(.a(new_n304), .b(new_n302), .c(new_n282), .d(new_n297), .o1(new_n305));
  aoi012aa1n12x5               g210(.a(new_n304), .b(new_n282), .c(new_n297), .o1(new_n306));
  nanp02aa1n03x5               g211(.a(new_n299), .b(new_n306), .o1(new_n307));
  aoi022aa1n02x7               g212(.a(new_n307), .b(new_n302), .c(new_n299), .d(new_n305), .o1(\s[27] ));
  inv000aa1d42x5               g213(.a(new_n301), .o1(new_n309));
  inv000aa1n02x5               g214(.a(new_n297), .o1(new_n310));
  aoai13aa1n04x5               g215(.a(new_n303), .b(new_n310), .c(new_n286), .d(new_n281), .o1(new_n311));
  aoai13aa1n02x5               g216(.a(new_n309), .b(new_n311), .c(new_n198), .d(new_n298), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[28] ), .b(\b[27] ), .out0(new_n313));
  norp02aa1n02x5               g218(.a(new_n313), .b(new_n300), .o1(new_n314));
  inv000aa1d42x5               g219(.a(new_n300), .o1(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n301), .c(new_n299), .d(new_n306), .o1(new_n316));
  aoi022aa1n03x5               g221(.a(new_n316), .b(new_n313), .c(new_n312), .d(new_n314), .o1(\s[28] ));
  and002aa1n06x5               g222(.a(new_n313), .b(new_n302), .o(new_n318));
  aoai13aa1n03x5               g223(.a(new_n318), .b(new_n311), .c(new_n198), .d(new_n298), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n318), .o1(new_n320));
  oao003aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .c(new_n315), .carry(new_n321));
  aoai13aa1n03x5               g226(.a(new_n321), .b(new_n320), .c(new_n299), .d(new_n306), .o1(new_n322));
  tech160nm_fixorc02aa1n02p5x5 g227(.a(\a[29] ), .b(\b[28] ), .out0(new_n323));
  norb02aa1n02x5               g228(.a(new_n321), .b(new_n323), .out0(new_n324));
  aoi022aa1n03x5               g229(.a(new_n322), .b(new_n323), .c(new_n319), .d(new_n324), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g231(.a(new_n313), .b(new_n323), .c(new_n302), .o(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n311), .c(new_n198), .d(new_n298), .o1(new_n328));
  inv000aa1n02x5               g233(.a(new_n327), .o1(new_n329));
  inv000aa1d42x5               g234(.a(\b[28] ), .o1(new_n330));
  inv000aa1d42x5               g235(.a(\a[29] ), .o1(new_n331));
  oaib12aa1n02x5               g236(.a(new_n321), .b(\b[28] ), .c(new_n331), .out0(new_n332));
  oaib12aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(\a[29] ), .out0(new_n333));
  aoai13aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n299), .d(new_n306), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n332), .c(new_n331), .d(new_n330), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n334), .b(new_n335), .c(new_n328), .d(new_n336), .o1(\s[30] ));
  nanb02aa1n02x5               g242(.a(\b[30] ), .b(\a[31] ), .out0(new_n338));
  nanb02aa1n02x5               g243(.a(\a[31] ), .b(\b[30] ), .out0(new_n339));
  nanp02aa1n02x5               g244(.a(new_n339), .b(new_n338), .o1(new_n340));
  nano22aa1n12x5               g245(.a(new_n320), .b(new_n323), .c(new_n335), .out0(new_n341));
  aoai13aa1n03x5               g246(.a(new_n341), .b(new_n311), .c(new_n198), .d(new_n298), .o1(new_n342));
  inv000aa1d42x5               g247(.a(new_n341), .o1(new_n343));
  norp02aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .o1(new_n344));
  aoi022aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n344), .b(new_n332), .c(new_n345), .o1(new_n346));
  aoai13aa1n03x5               g251(.a(new_n346), .b(new_n343), .c(new_n299), .d(new_n306), .o1(new_n347));
  oai112aa1n02x5               g252(.a(new_n338), .b(new_n339), .c(\b[29] ), .d(\a[30] ), .o1(new_n348));
  aoi012aa1n02x5               g253(.a(new_n348), .b(new_n332), .c(new_n345), .o1(new_n349));
  aoi022aa1n03x5               g254(.a(new_n347), .b(new_n340), .c(new_n342), .d(new_n349), .o1(\s[31] ));
  norb02aa1n02x5               g255(.a(new_n104), .b(new_n106), .out0(new_n351));
  xobna2aa1n03x5               g256(.a(new_n351), .b(new_n131), .c(new_n100), .out0(\s[3] ));
  aoai13aa1n02x5               g257(.a(new_n351), .b(new_n102), .c(\a[2] ), .d(\b[1] ), .o1(new_n353));
  nanb02aa1n02x5               g258(.a(new_n108), .b(new_n105), .out0(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n354), .b(new_n353), .c(new_n104), .out0(\s[4] ));
  oai022aa1n02x5               g260(.a(\a[4] ), .b(\b[3] ), .c(\b[4] ), .d(\a[5] ), .o1(new_n356));
  aoi122aa1n06x5               g261(.a(new_n356), .b(\a[5] ), .c(\b[4] ), .d(new_n106), .e(new_n105), .o1(new_n357));
  oai013aa1n03x5               g262(.a(new_n357), .b(new_n102), .c(new_n107), .d(new_n103), .o1(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n118), .c(new_n132), .d(new_n109), .o1(\s[5] ));
  xnbna2aa1n03x5               g264(.a(new_n114), .b(new_n358), .c(new_n117), .out0(\s[6] ));
  aob012aa1n03x5               g265(.a(new_n115), .b(new_n358), .c(new_n117), .out0(new_n361));
  xnbna2aa1n03x5               g266(.a(new_n111), .b(new_n361), .c(new_n113), .out0(\s[7] ));
  tech160nm_fiaoi012aa1n05x5   g267(.a(new_n111), .b(new_n361), .c(new_n113), .o1(new_n363));
  aoai13aa1n02x5               g268(.a(new_n119), .b(new_n363), .c(\b[6] ), .d(\a[7] ), .o1(new_n364));
  aoi012aa1n02x5               g269(.a(new_n119), .b(\a[7] ), .c(\b[6] ), .o1(new_n365));
  oaib12aa1n02x5               g270(.a(new_n364), .b(new_n363), .c(new_n365), .out0(\s[8] ));
  aoi112aa1n02x5               g271(.a(new_n124), .b(new_n125), .c(new_n120), .d(new_n110), .o1(new_n367));
  aoi012aa1n02x5               g272(.a(new_n367), .b(new_n135), .c(new_n125), .o1(\s[9] ));
endmodule


