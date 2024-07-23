// Benchmark "adder" written by ABC on Wed Jul 17 14:54:10 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n160, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n175, new_n176, new_n177, new_n178, new_n179,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n188,
    new_n189, new_n190, new_n191, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n204, new_n205,
    new_n206, new_n207, new_n208, new_n209, new_n210, new_n212, new_n213,
    new_n214, new_n215, new_n216, new_n217, new_n218, new_n219, new_n221,
    new_n222, new_n223, new_n224, new_n225, new_n226, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n242, new_n243, new_n244,
    new_n245, new_n246, new_n247, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n262, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n292, new_n293, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n302, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n312, new_n314, new_n315, new_n316, new_n318,
    new_n319, new_n321, new_n322, new_n325;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[2] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[1] ), .o1(new_n98));
  nand22aa1n12x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  oaoi03aa1n12x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor022aa1n08x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nand02aa1n03x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor042aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n09x5               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  aoi012aa1n02x7               g010(.a(new_n101), .b(new_n103), .c(new_n102), .o1(new_n106));
  oai012aa1n12x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .o1(new_n107));
  nor022aa1n16x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand02aa1n12x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor022aa1n16x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand42aa1n02x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xnrc02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nor042aa1d18x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n04x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nanb02aa1n02x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  nor043aa1n06x5               g021(.a(new_n112), .b(new_n113), .c(new_n116), .o1(new_n117));
  xorc02aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  inv020aa1d32x5               g023(.a(\b[5] ), .o1(new_n119));
  nanb02aa1d24x5               g024(.a(\a[6] ), .b(new_n119), .out0(new_n120));
  oai112aa1n06x5               g025(.a(new_n120), .b(new_n109), .c(\b[4] ), .d(\a[5] ), .o1(new_n121));
  nano22aa1n03x7               g026(.a(new_n114), .b(new_n109), .c(new_n115), .out0(new_n122));
  nand03aa1n02x5               g027(.a(new_n122), .b(new_n118), .c(new_n121), .o1(new_n123));
  inv020aa1n02x5               g028(.a(new_n114), .o1(new_n124));
  oao003aa1n02x5               g029(.a(\a[8] ), .b(\b[7] ), .c(new_n124), .carry(new_n125));
  nand22aa1n06x5               g030(.a(new_n123), .b(new_n125), .o1(new_n126));
  aoi012aa1n02x5               g031(.a(new_n126), .b(new_n107), .c(new_n117), .o1(new_n127));
  oaoi03aa1n02x5               g032(.a(\a[9] ), .b(\b[8] ), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor022aa1n16x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nor002aa1n10x5               g036(.a(\b[8] ), .b(\a[9] ), .o1(new_n132));
  nanp02aa1n06x5               g037(.a(\b[8] ), .b(\a[9] ), .o1(new_n133));
  nano23aa1n06x5               g038(.a(new_n130), .b(new_n132), .c(new_n133), .d(new_n131), .out0(new_n134));
  aoai13aa1n02x5               g039(.a(new_n134), .b(new_n126), .c(new_n107), .d(new_n117), .o1(new_n135));
  oai012aa1n12x5               g040(.a(new_n131), .b(new_n132), .c(new_n130), .o1(new_n136));
  nor002aa1d32x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnbna2aa1n03x5               g044(.a(new_n139), .b(new_n135), .c(new_n136), .out0(\s[11] ));
  inv000aa1d42x5               g045(.a(new_n137), .o1(new_n141));
  aob012aa1n02x5               g046(.a(new_n139), .b(new_n135), .c(new_n136), .out0(new_n142));
  nor002aa1d24x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1n20x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  nona23aa1n02x4               g050(.a(new_n142), .b(new_n144), .c(new_n143), .d(new_n137), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n141), .d(new_n142), .o1(\s[12] ));
  nano23aa1n06x5               g052(.a(new_n137), .b(new_n143), .c(new_n144), .d(new_n138), .out0(new_n148));
  nand22aa1n09x5               g053(.a(new_n148), .b(new_n134), .o1(new_n149));
  inv000aa1d42x5               g054(.a(new_n149), .o1(new_n150));
  aoai13aa1n06x5               g055(.a(new_n150), .b(new_n126), .c(new_n107), .d(new_n117), .o1(new_n151));
  nona23aa1d18x5               g056(.a(new_n144), .b(new_n138), .c(new_n137), .d(new_n143), .out0(new_n152));
  tech160nm_fioai012aa1n03p5x5 g057(.a(new_n144), .b(new_n143), .c(new_n137), .o1(new_n153));
  oai012aa1d24x5               g058(.a(new_n153), .b(new_n152), .c(new_n136), .o1(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  nor002aa1n04x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  nand42aa1d28x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  norb02aa1n02x5               g062(.a(new_n157), .b(new_n156), .out0(new_n158));
  xnbna2aa1n03x5               g063(.a(new_n158), .b(new_n151), .c(new_n155), .out0(\s[13] ));
  nanp02aa1n06x5               g064(.a(new_n151), .b(new_n155), .o1(new_n160));
  nor022aa1n08x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n16x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nanb02aa1n02x5               g067(.a(new_n161), .b(new_n162), .out0(new_n163));
  aoai13aa1n02x5               g068(.a(new_n163), .b(new_n156), .c(new_n160), .d(new_n157), .o1(new_n164));
  nona22aa1n02x4               g069(.a(new_n162), .b(new_n161), .c(new_n156), .out0(new_n165));
  aoai13aa1n02x5               g070(.a(new_n164), .b(new_n165), .c(new_n158), .d(new_n160), .o1(\s[14] ));
  nano23aa1n06x5               g071(.a(new_n156), .b(new_n161), .c(new_n162), .d(new_n157), .out0(new_n167));
  oa0012aa1n02x5               g072(.a(new_n162), .b(new_n161), .c(new_n156), .o(new_n168));
  nor042aa1n09x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  nand42aa1n03x5               g074(.a(\b[14] ), .b(\a[15] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n160), .d(new_n167), .o1(new_n172));
  aoi112aa1n02x5               g077(.a(new_n171), .b(new_n168), .c(new_n160), .d(new_n167), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n172), .b(new_n173), .out0(\s[15] ));
  inv000aa1d42x5               g079(.a(new_n169), .o1(new_n175));
  nor042aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .o1(new_n176));
  nanp02aa1n04x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  nona23aa1n03x5               g083(.a(new_n172), .b(new_n177), .c(new_n176), .d(new_n169), .out0(new_n179));
  aoai13aa1n03x5               g084(.a(new_n179), .b(new_n178), .c(new_n172), .d(new_n175), .o1(\s[16] ));
  nano23aa1n03x7               g085(.a(new_n169), .b(new_n176), .c(new_n177), .d(new_n170), .out0(new_n181));
  nano22aa1n06x5               g086(.a(new_n149), .b(new_n167), .c(new_n181), .out0(new_n182));
  aoai13aa1n12x5               g087(.a(new_n182), .b(new_n126), .c(new_n107), .d(new_n117), .o1(new_n183));
  aoai13aa1n06x5               g088(.a(new_n181), .b(new_n168), .c(new_n154), .d(new_n167), .o1(new_n184));
  oai012aa1n02x5               g089(.a(new_n177), .b(new_n176), .c(new_n169), .o1(new_n185));
  nand23aa1d12x5               g090(.a(new_n183), .b(new_n184), .c(new_n185), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g092(.a(\a[18] ), .o1(new_n188));
  inv020aa1d32x5               g093(.a(\a[17] ), .o1(new_n189));
  inv000aa1d42x5               g094(.a(\b[16] ), .o1(new_n190));
  tech160nm_fioaoi03aa1n03p5x5 g095(.a(new_n189), .b(new_n190), .c(new_n186), .o1(new_n191));
  xorb03aa1n02x5               g096(.a(new_n191), .b(\b[17] ), .c(new_n188), .out0(\s[18] ));
  xroi22aa1d06x4               g097(.a(new_n189), .b(\b[16] ), .c(new_n188), .d(\b[17] ), .out0(new_n193));
  oai022aa1d24x5               g098(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n194));
  oaib12aa1n09x5               g099(.a(new_n194), .b(new_n188), .c(\b[17] ), .out0(new_n195));
  inv020aa1n04x5               g100(.a(new_n195), .o1(new_n196));
  nor042aa1d18x5               g101(.a(\b[18] ), .b(\a[19] ), .o1(new_n197));
  nand42aa1n04x5               g102(.a(\b[18] ), .b(\a[19] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n196), .c(new_n186), .d(new_n193), .o1(new_n200));
  aoi112aa1n02x5               g105(.a(new_n199), .b(new_n196), .c(new_n186), .d(new_n193), .o1(new_n201));
  norb02aa1n03x4               g106(.a(new_n200), .b(new_n201), .out0(\s[19] ));
  xnrc02aa1n02x5               g107(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g108(.a(new_n197), .o1(new_n204));
  nor042aa1n04x5               g109(.a(\b[19] ), .b(\a[20] ), .o1(new_n205));
  nand42aa1n06x5               g110(.a(\b[19] ), .b(\a[20] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  inv000aa1n02x5               g112(.a(new_n206), .o1(new_n208));
  oai022aa1n02x5               g113(.a(\a[19] ), .b(\b[18] ), .c(\b[19] ), .d(\a[20] ), .o1(new_n209));
  nona22aa1n03x5               g114(.a(new_n200), .b(new_n208), .c(new_n209), .out0(new_n210));
  aoai13aa1n03x5               g115(.a(new_n210), .b(new_n207), .c(new_n204), .d(new_n200), .o1(\s[20] ));
  nona23aa1n03x5               g116(.a(new_n206), .b(new_n198), .c(new_n197), .d(new_n205), .out0(new_n212));
  norb02aa1n02x5               g117(.a(new_n193), .b(new_n212), .out0(new_n213));
  oab012aa1n03x5               g118(.a(new_n208), .b(new_n197), .c(new_n205), .out0(new_n214));
  inv030aa1n02x5               g119(.a(new_n214), .o1(new_n215));
  oai012aa1n06x5               g120(.a(new_n215), .b(new_n212), .c(new_n195), .o1(new_n216));
  xorc02aa1n02x5               g121(.a(\a[21] ), .b(\b[20] ), .out0(new_n217));
  aoai13aa1n06x5               g122(.a(new_n217), .b(new_n216), .c(new_n186), .d(new_n213), .o1(new_n218));
  aoi112aa1n02x5               g123(.a(new_n217), .b(new_n216), .c(new_n186), .d(new_n213), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n218), .b(new_n219), .out0(\s[21] ));
  nor042aa1n03x5               g125(.a(\b[20] ), .b(\a[21] ), .o1(new_n221));
  inv040aa1n03x5               g126(.a(new_n221), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[22] ), .b(\b[21] ), .out0(new_n223));
  and002aa1n02x5               g128(.a(\b[21] ), .b(\a[22] ), .o(new_n224));
  oai022aa1n02x5               g129(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n225));
  nona22aa1n03x5               g130(.a(new_n218), .b(new_n224), .c(new_n225), .out0(new_n226));
  aoai13aa1n03x5               g131(.a(new_n226), .b(new_n223), .c(new_n222), .d(new_n218), .o1(\s[22] ));
  nano23aa1n09x5               g132(.a(new_n197), .b(new_n205), .c(new_n206), .d(new_n198), .out0(new_n228));
  inv000aa1d42x5               g133(.a(\a[21] ), .o1(new_n229));
  inv000aa1d42x5               g134(.a(\a[22] ), .o1(new_n230));
  xroi22aa1d06x4               g135(.a(new_n229), .b(\b[20] ), .c(new_n230), .d(\b[21] ), .out0(new_n231));
  nanp03aa1d12x5               g136(.a(new_n231), .b(new_n193), .c(new_n228), .o1(new_n232));
  inv000aa1d42x5               g137(.a(new_n232), .o1(new_n233));
  aoai13aa1n06x5               g138(.a(new_n231), .b(new_n214), .c(new_n228), .d(new_n196), .o1(new_n234));
  oaoi03aa1n09x5               g139(.a(\a[22] ), .b(\b[21] ), .c(new_n222), .o1(new_n235));
  inv000aa1n02x5               g140(.a(new_n235), .o1(new_n236));
  nanp02aa1n02x5               g141(.a(new_n234), .b(new_n236), .o1(new_n237));
  xorc02aa1n12x5               g142(.a(\a[23] ), .b(\b[22] ), .out0(new_n238));
  aoai13aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n186), .d(new_n233), .o1(new_n239));
  aoi112aa1n02x5               g144(.a(new_n238), .b(new_n237), .c(new_n186), .d(new_n233), .o1(new_n240));
  norb02aa1n03x4               g145(.a(new_n239), .b(new_n240), .out0(\s[23] ));
  norp02aa1n02x5               g146(.a(\b[22] ), .b(\a[23] ), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n242), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[24] ), .b(\b[23] ), .out0(new_n244));
  and002aa1n02x5               g149(.a(\b[23] ), .b(\a[24] ), .o(new_n245));
  oai022aa1n02x5               g150(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n246));
  nona22aa1n03x5               g151(.a(new_n239), .b(new_n245), .c(new_n246), .out0(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n244), .c(new_n243), .d(new_n239), .o1(\s[24] ));
  nanp02aa1n02x5               g153(.a(new_n244), .b(new_n238), .o1(new_n249));
  nano32aa1n02x5               g154(.a(new_n249), .b(new_n231), .c(new_n193), .d(new_n228), .out0(new_n250));
  aob012aa1n02x5               g155(.a(new_n246), .b(\b[23] ), .c(\a[24] ), .out0(new_n251));
  aoai13aa1n04x5               g156(.a(new_n251), .b(new_n249), .c(new_n234), .d(new_n236), .o1(new_n252));
  tech160nm_fixorc02aa1n04x5   g157(.a(\a[25] ), .b(\b[24] ), .out0(new_n253));
  aoai13aa1n06x5               g158(.a(new_n253), .b(new_n252), .c(new_n186), .d(new_n250), .o1(new_n254));
  aoi112aa1n02x5               g159(.a(new_n253), .b(new_n252), .c(new_n186), .d(new_n250), .o1(new_n255));
  norb02aa1n03x4               g160(.a(new_n254), .b(new_n255), .out0(\s[25] ));
  norp02aa1n02x5               g161(.a(\b[24] ), .b(\a[25] ), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n257), .o1(new_n258));
  xorc02aa1n02x5               g163(.a(\a[26] ), .b(\b[25] ), .out0(new_n259));
  and002aa1n02x5               g164(.a(\b[25] ), .b(\a[26] ), .o(new_n260));
  oai022aa1n02x5               g165(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n261));
  nona22aa1n03x5               g166(.a(new_n254), .b(new_n260), .c(new_n261), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n259), .c(new_n258), .d(new_n254), .o1(\s[26] ));
  inv030aa1n02x5               g168(.a(new_n249), .o1(new_n264));
  and002aa1n02x5               g169(.a(new_n259), .b(new_n253), .o(new_n265));
  nano22aa1n12x5               g170(.a(new_n232), .b(new_n264), .c(new_n265), .out0(new_n266));
  aoai13aa1n03x5               g171(.a(new_n264), .b(new_n235), .c(new_n216), .d(new_n231), .o1(new_n267));
  inv000aa1n02x5               g172(.a(new_n265), .o1(new_n268));
  aob012aa1n02x5               g173(.a(new_n261), .b(\b[25] ), .c(\a[26] ), .out0(new_n269));
  aoai13aa1n06x5               g174(.a(new_n269), .b(new_n268), .c(new_n267), .d(new_n251), .o1(new_n270));
  xorc02aa1n02x5               g175(.a(\a[27] ), .b(\b[26] ), .out0(new_n271));
  aoai13aa1n06x5               g176(.a(new_n271), .b(new_n270), .c(new_n186), .d(new_n266), .o1(new_n272));
  inv020aa1n03x5               g177(.a(new_n266), .o1(new_n273));
  aoi013aa1n02x4               g178(.a(new_n273), .b(new_n183), .c(new_n184), .d(new_n185), .o1(new_n274));
  nanb02aa1n02x5               g179(.a(new_n271), .b(new_n269), .out0(new_n275));
  aoi112aa1n02x5               g180(.a(new_n274), .b(new_n275), .c(new_n252), .d(new_n265), .o1(new_n276));
  norb02aa1n02x7               g181(.a(new_n272), .b(new_n276), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[27] ), .b(\a[28] ), .o1(new_n278));
  nanp02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .o1(new_n279));
  norb02aa1n02x5               g184(.a(new_n279), .b(new_n278), .out0(new_n280));
  norp02aa1n02x5               g185(.a(\b[26] ), .b(\a[27] ), .o1(new_n281));
  oaoi13aa1n03x5               g186(.a(new_n281), .b(new_n271), .c(new_n274), .d(new_n270), .o1(new_n282));
  oai112aa1n03x5               g187(.a(new_n272), .b(new_n280), .c(\b[26] ), .d(\a[27] ), .o1(new_n283));
  oaih12aa1n02x5               g188(.a(new_n283), .b(new_n282), .c(new_n280), .o1(\s[28] ));
  and002aa1n06x5               g189(.a(new_n271), .b(new_n280), .o(new_n285));
  aoai13aa1n02x5               g190(.a(new_n285), .b(new_n270), .c(new_n186), .d(new_n266), .o1(new_n286));
  nand02aa1d06x5               g191(.a(new_n186), .b(new_n266), .o1(new_n287));
  aobi12aa1n02x7               g192(.a(new_n269), .b(new_n252), .c(new_n265), .out0(new_n288));
  inv000aa1d42x5               g193(.a(new_n285), .o1(new_n289));
  oai012aa1n02x5               g194(.a(new_n279), .b(new_n278), .c(new_n281), .o1(new_n290));
  aoai13aa1n03x5               g195(.a(new_n290), .b(new_n289), .c(new_n287), .d(new_n288), .o1(new_n291));
  xorc02aa1n02x5               g196(.a(\a[29] ), .b(\b[28] ), .out0(new_n292));
  norb02aa1n02x5               g197(.a(new_n290), .b(new_n292), .out0(new_n293));
  aoi022aa1n03x5               g198(.a(new_n291), .b(new_n292), .c(new_n286), .d(new_n293), .o1(\s[29] ));
  xorb03aa1n02x5               g199(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g200(.a(new_n271), .b(new_n292), .c(new_n280), .o(new_n296));
  aoai13aa1n02x5               g201(.a(new_n296), .b(new_n270), .c(new_n186), .d(new_n266), .o1(new_n297));
  inv000aa1d42x5               g202(.a(new_n296), .o1(new_n298));
  oao003aa1n02x5               g203(.a(\a[29] ), .b(\b[28] ), .c(new_n290), .carry(new_n299));
  aoai13aa1n03x5               g204(.a(new_n299), .b(new_n298), .c(new_n287), .d(new_n288), .o1(new_n300));
  xorc02aa1n02x5               g205(.a(\a[30] ), .b(\b[29] ), .out0(new_n301));
  norb02aa1n02x5               g206(.a(new_n299), .b(new_n301), .out0(new_n302));
  aoi022aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n297), .d(new_n302), .o1(\s[30] ));
  nano22aa1n09x5               g208(.a(new_n289), .b(new_n292), .c(new_n301), .out0(new_n304));
  aoai13aa1n02x5               g209(.a(new_n304), .b(new_n270), .c(new_n186), .d(new_n266), .o1(new_n305));
  inv000aa1d42x5               g210(.a(new_n304), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n299), .carry(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n287), .d(new_n288), .o1(new_n308));
  xorc02aa1n02x5               g213(.a(\a[31] ), .b(\b[30] ), .out0(new_n309));
  norb02aa1n02x5               g214(.a(new_n307), .b(new_n309), .out0(new_n310));
  aoi022aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n305), .d(new_n310), .o1(\s[31] ));
  norb02aa1n02x5               g216(.a(new_n104), .b(new_n103), .out0(new_n312));
  xnrc02aa1n02x5               g217(.a(new_n100), .b(new_n312), .out0(\s[3] ));
  inv000aa1d42x5               g218(.a(new_n100), .o1(new_n314));
  obai22aa1n02x7               g219(.a(new_n102), .b(new_n101), .c(\a[3] ), .d(\b[2] ), .out0(new_n315));
  aoi012aa1n02x5               g220(.a(new_n315), .b(new_n314), .c(new_n312), .o1(new_n316));
  aoib12aa1n02x5               g221(.a(new_n316), .b(new_n107), .c(new_n101), .out0(\s[4] ));
  nanb02aa1n02x5               g222(.a(new_n105), .b(new_n314), .out0(new_n318));
  norb02aa1n02x5               g223(.a(new_n111), .b(new_n110), .out0(new_n319));
  xnbna2aa1n03x5               g224(.a(new_n319), .b(new_n318), .c(new_n106), .out0(\s[5] ));
  aoi012aa1n02x5               g225(.a(new_n110), .b(new_n107), .c(new_n111), .o1(new_n321));
  tech160nm_fiao0012aa1n02p5x5 g226(.a(new_n121), .b(new_n107), .c(new_n319), .o(new_n322));
  aoai13aa1n02x5               g227(.a(new_n322), .b(new_n321), .c(new_n120), .d(new_n109), .o1(\s[6] ));
  xnbna2aa1n03x5               g228(.a(new_n116), .b(new_n322), .c(new_n109), .out0(\s[7] ));
  aoai13aa1n02x5               g229(.a(new_n122), .b(new_n121), .c(new_n107), .d(new_n319), .o1(new_n325));
  xnbna2aa1n03x5               g230(.a(new_n118), .b(new_n325), .c(new_n124), .out0(\s[8] ));
  xnrb03aa1n02x5               g231(.a(new_n127), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


