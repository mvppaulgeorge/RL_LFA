// Benchmark "adder" written by ABC on Wed Jul 17 21:32:25 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n131,
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n176, new_n177,
    new_n178, new_n179, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n216,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n224,
    new_n225, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n249,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n301, new_n302,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n322, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n356, new_n358, new_n361,
    new_n362, new_n364, new_n366, new_n367, new_n368, new_n370, new_n371;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n02x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand42aa1n10x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand02aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norb03aa1n03x5               g004(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n100));
  tech160nm_fixnrc02aa1n04x5   g005(.a(\b[3] ), .b(\a[4] ), .out0(new_n101));
  nor022aa1n16x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n08x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb03aa1n03x5               g008(.a(new_n102), .b(new_n103), .c(new_n98), .out0(new_n104));
  aoi112aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .c(\a[4] ), .d(\b[3] ), .o1(new_n105));
  oab012aa1n02x5               g010(.a(new_n105), .b(\a[4] ), .c(\b[3] ), .out0(new_n106));
  oai013aa1n03x5               g011(.a(new_n106), .b(new_n100), .c(new_n104), .d(new_n101), .o1(new_n107));
  nor042aa1n09x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  nand42aa1n02x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor022aa1n06x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  nona23aa1n02x4               g016(.a(new_n111), .b(new_n109), .c(new_n108), .d(new_n110), .out0(new_n112));
  xorc02aa1n12x5               g017(.a(\a[8] ), .b(\b[7] ), .out0(new_n113));
  norp02aa1n09x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n02x7               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nano22aa1n03x7               g021(.a(new_n112), .b(new_n113), .c(new_n116), .out0(new_n117));
  nano22aa1n03x7               g022(.a(new_n114), .b(new_n109), .c(new_n115), .out0(new_n118));
  oai022aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  nanp03aa1n02x5               g024(.a(new_n118), .b(new_n113), .c(new_n119), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\a[8] ), .o1(new_n121));
  inv000aa1d42x5               g026(.a(\b[7] ), .o1(new_n122));
  oaoi03aa1n03x5               g027(.a(new_n121), .b(new_n122), .c(new_n114), .o1(new_n123));
  nanp02aa1n02x5               g028(.a(new_n120), .b(new_n123), .o1(new_n124));
  nor042aa1n06x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  nand42aa1n16x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n124), .c(new_n107), .d(new_n117), .o1(new_n128));
  oai012aa1n02x5               g033(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .o1(new_n129));
  tech160nm_finor002aa1n05x5   g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1n20x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  aoib12aa1n02x5               g037(.a(new_n125), .b(new_n131), .c(new_n130), .out0(new_n133));
  aoi022aa1n02x5               g038(.a(new_n129), .b(new_n132), .c(new_n128), .d(new_n133), .o1(\s[10] ));
  nano23aa1n06x5               g039(.a(new_n125), .b(new_n130), .c(new_n131), .d(new_n126), .out0(new_n135));
  aoai13aa1n02x5               g040(.a(new_n135), .b(new_n124), .c(new_n107), .d(new_n117), .o1(new_n136));
  oai012aa1n02x5               g041(.a(new_n131), .b(new_n130), .c(new_n125), .o1(new_n137));
  nor022aa1n08x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  norb02aa1n06x5               g044(.a(new_n139), .b(new_n138), .out0(new_n140));
  xnbna2aa1n03x5               g045(.a(new_n140), .b(new_n136), .c(new_n137), .out0(\s[11] ));
  aob012aa1n02x5               g046(.a(new_n140), .b(new_n136), .c(new_n137), .out0(new_n142));
  nor042aa1d18x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand02aa1d12x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  norb02aa1n15x5               g049(.a(new_n144), .b(new_n143), .out0(new_n145));
  inv000aa1d42x5               g050(.a(new_n143), .o1(new_n146));
  aoi012aa1n02x5               g051(.a(new_n138), .b(new_n146), .c(new_n144), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n142), .b(\b[10] ), .c(\a[11] ), .o1(new_n148));
  aoi022aa1n02x5               g053(.a(new_n148), .b(new_n145), .c(new_n142), .d(new_n147), .o1(\s[12] ));
  nona22aa1n02x4               g054(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n150));
  nano22aa1n02x4               g055(.a(new_n102), .b(new_n98), .c(new_n103), .out0(new_n151));
  nanb03aa1n06x5               g056(.a(new_n101), .b(new_n151), .c(new_n150), .out0(new_n152));
  norb02aa1n09x5               g057(.a(new_n109), .b(new_n108), .out0(new_n153));
  nanb02aa1n02x5               g058(.a(new_n110), .b(new_n111), .out0(new_n154));
  nanb02aa1n02x5               g059(.a(new_n114), .b(new_n115), .out0(new_n155));
  nona23aa1n03x5               g060(.a(new_n113), .b(new_n153), .c(new_n155), .d(new_n154), .out0(new_n156));
  inv000aa1n02x5               g061(.a(new_n123), .o1(new_n157));
  aoi013aa1n02x4               g062(.a(new_n157), .b(new_n118), .c(new_n113), .d(new_n119), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n156), .c(new_n152), .d(new_n106), .o1(new_n159));
  nand23aa1d12x5               g064(.a(new_n135), .b(new_n140), .c(new_n145), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  nano22aa1n03x7               g066(.a(new_n143), .b(new_n139), .c(new_n144), .out0(new_n162));
  tech160nm_fioai012aa1n03p5x5 g067(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .o1(new_n163));
  oab012aa1n09x5               g068(.a(new_n163), .b(new_n125), .c(new_n130), .out0(new_n164));
  nand02aa1d06x5               g069(.a(new_n164), .b(new_n162), .o1(new_n165));
  aoi012aa1d24x5               g070(.a(new_n143), .b(new_n138), .c(new_n144), .o1(new_n166));
  nand42aa1n06x5               g071(.a(new_n165), .b(new_n166), .o1(new_n167));
  norp02aa1n24x5               g072(.a(\b[12] ), .b(\a[13] ), .o1(new_n168));
  nand42aa1n03x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  aoai13aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n159), .d(new_n161), .o1(new_n171));
  inv000aa1d42x5               g076(.a(new_n166), .o1(new_n172));
  nona22aa1n02x4               g077(.a(new_n165), .b(new_n172), .c(new_n170), .out0(new_n173));
  aoi012aa1n02x5               g078(.a(new_n173), .b(new_n159), .c(new_n161), .o1(new_n174));
  norb02aa1n02x5               g079(.a(new_n171), .b(new_n174), .out0(\s[13] ));
  inv000aa1d42x5               g080(.a(new_n168), .o1(new_n176));
  nor022aa1n08x5               g081(.a(\b[13] ), .b(\a[14] ), .o1(new_n177));
  tech160nm_finand02aa1n03p5x5 g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  xnbna2aa1n03x5               g084(.a(new_n179), .b(new_n171), .c(new_n176), .out0(\s[14] ));
  nano23aa1d12x5               g085(.a(new_n168), .b(new_n177), .c(new_n178), .d(new_n169), .out0(new_n181));
  nand03aa1n02x5               g086(.a(new_n159), .b(new_n161), .c(new_n181), .o1(new_n182));
  tech160nm_fioai012aa1n05x5   g087(.a(new_n178), .b(new_n177), .c(new_n168), .o1(new_n183));
  inv000aa1n02x5               g088(.a(new_n183), .o1(new_n184));
  aoi012aa1n02x5               g089(.a(new_n184), .b(new_n167), .c(new_n181), .o1(new_n185));
  norp02aa1n06x5               g090(.a(\b[14] ), .b(\a[15] ), .o1(new_n186));
  nand42aa1n16x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  aob012aa1n03x5               g093(.a(new_n188), .b(new_n182), .c(new_n185), .out0(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n184), .c(new_n167), .d(new_n181), .o1(new_n190));
  aobi12aa1n02x5               g095(.a(new_n189), .b(new_n190), .c(new_n182), .out0(\s[15] ));
  nor002aa1n04x5               g096(.a(\b[15] ), .b(\a[16] ), .o1(new_n192));
  nand42aa1n16x5               g097(.a(\b[15] ), .b(\a[16] ), .o1(new_n193));
  norb02aa1n02x5               g098(.a(new_n193), .b(new_n192), .out0(new_n194));
  aoib12aa1n02x5               g099(.a(new_n186), .b(new_n193), .c(new_n192), .out0(new_n195));
  oaih12aa1n02x5               g100(.a(new_n189), .b(\b[14] ), .c(\a[15] ), .o1(new_n196));
  aoi022aa1n02x5               g101(.a(new_n196), .b(new_n194), .c(new_n189), .d(new_n195), .o1(\s[16] ));
  nano23aa1d15x5               g102(.a(new_n186), .b(new_n192), .c(new_n193), .d(new_n187), .out0(new_n198));
  nano22aa1d15x5               g103(.a(new_n160), .b(new_n181), .c(new_n198), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n124), .c(new_n107), .d(new_n117), .o1(new_n200));
  xorc02aa1n02x5               g105(.a(\a[17] ), .b(\b[16] ), .out0(new_n201));
  aoai13aa1n06x5               g106(.a(new_n198), .b(new_n184), .c(new_n167), .d(new_n181), .o1(new_n202));
  aoi012aa1n02x5               g107(.a(new_n192), .b(new_n186), .c(new_n193), .o1(new_n203));
  nano22aa1n02x4               g108(.a(new_n201), .b(new_n202), .c(new_n203), .out0(new_n204));
  nanp03aa1d12x5               g109(.a(new_n200), .b(new_n202), .c(new_n203), .o1(new_n205));
  aoi022aa1n02x5               g110(.a(new_n204), .b(new_n200), .c(new_n201), .d(new_n205), .o1(\s[17] ));
  nanp02aa1n02x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  aoai13aa1n09x5               g112(.a(new_n181), .b(new_n172), .c(new_n164), .d(new_n162), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n198), .o1(new_n209));
  aoai13aa1n06x5               g114(.a(new_n203), .b(new_n209), .c(new_n208), .d(new_n183), .o1(new_n210));
  aoai13aa1n02x5               g115(.a(new_n207), .b(new_n210), .c(new_n199), .d(new_n159), .o1(new_n211));
  inv000aa1d42x5               g116(.a(\a[17] ), .o1(new_n212));
  oaib12aa1n02x5               g117(.a(new_n211), .b(\b[16] ), .c(new_n212), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[18] ), .b(\b[17] ), .out0(new_n214));
  nor002aa1n04x5               g119(.a(\b[16] ), .b(\a[17] ), .o1(new_n215));
  norp02aa1n02x5               g120(.a(new_n214), .b(new_n215), .o1(new_n216));
  aoi022aa1n02x5               g121(.a(new_n213), .b(new_n214), .c(new_n211), .d(new_n216), .o1(\s[18] ));
  inv000aa1d42x5               g122(.a(\a[18] ), .o1(new_n218));
  xroi22aa1d04x5               g123(.a(new_n212), .b(\b[16] ), .c(new_n218), .d(\b[17] ), .out0(new_n219));
  aoai13aa1n03x5               g124(.a(new_n219), .b(new_n210), .c(new_n159), .d(new_n199), .o1(new_n220));
  nanp02aa1n02x5               g125(.a(\b[17] ), .b(\a[18] ), .o1(new_n221));
  oab012aa1n02x5               g126(.a(new_n215), .b(\a[18] ), .c(\b[17] ), .out0(new_n222));
  norb02aa1n02x5               g127(.a(new_n221), .b(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  xorc02aa1n12x5               g129(.a(\a[19] ), .b(\b[18] ), .out0(new_n225));
  xnbna2aa1n03x5               g130(.a(new_n225), .b(new_n220), .c(new_n224), .out0(\s[19] ));
  xnrc02aa1n02x5               g131(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g132(.a(new_n225), .b(new_n223), .c(new_n205), .d(new_n219), .o1(new_n228));
  nor042aa1n04x5               g133(.a(\b[19] ), .b(\a[20] ), .o1(new_n229));
  and002aa1n12x5               g134(.a(\b[19] ), .b(\a[20] ), .o(new_n230));
  nor042aa1n02x5               g135(.a(new_n230), .b(new_n229), .o1(new_n231));
  nor042aa1n04x5               g136(.a(\b[18] ), .b(\a[19] ), .o1(new_n232));
  oab012aa1n02x4               g137(.a(new_n232), .b(new_n230), .c(new_n229), .out0(new_n233));
  inv000aa1n02x5               g138(.a(new_n232), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n225), .o1(new_n235));
  aoai13aa1n02x5               g140(.a(new_n234), .b(new_n235), .c(new_n220), .d(new_n224), .o1(new_n236));
  aoi022aa1n03x5               g141(.a(new_n236), .b(new_n231), .c(new_n228), .d(new_n233), .o1(\s[20] ));
  nano32aa1n03x7               g142(.a(new_n235), .b(new_n214), .c(new_n201), .d(new_n231), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n210), .c(new_n159), .d(new_n199), .o1(new_n239));
  aoi112aa1n06x5               g144(.a(new_n230), .b(new_n229), .c(\a[19] ), .d(\b[18] ), .o1(new_n240));
  nona23aa1n09x5               g145(.a(new_n240), .b(new_n221), .c(new_n222), .d(new_n232), .out0(new_n241));
  oab012aa1n06x5               g146(.a(new_n229), .b(new_n234), .c(new_n230), .out0(new_n242));
  nanp02aa1n06x5               g147(.a(new_n241), .b(new_n242), .o1(new_n243));
  xorc02aa1n02x5               g148(.a(\a[21] ), .b(\b[20] ), .out0(new_n244));
  aoai13aa1n06x5               g149(.a(new_n244), .b(new_n243), .c(new_n205), .d(new_n238), .o1(new_n245));
  inv000aa1d42x5               g150(.a(new_n244), .o1(new_n246));
  and003aa1n02x5               g151(.a(new_n241), .b(new_n246), .c(new_n242), .o(new_n247));
  aobi12aa1n03x7               g152(.a(new_n245), .b(new_n247), .c(new_n239), .out0(\s[21] ));
  xorc02aa1n02x5               g153(.a(\a[22] ), .b(\b[21] ), .out0(new_n249));
  nor002aa1n20x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  norp02aa1n02x5               g155(.a(new_n249), .b(new_n250), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n243), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n250), .o1(new_n253));
  aoai13aa1n02x5               g158(.a(new_n253), .b(new_n246), .c(new_n239), .d(new_n252), .o1(new_n254));
  aoi022aa1n03x5               g159(.a(new_n254), .b(new_n249), .c(new_n245), .d(new_n251), .o1(\s[22] ));
  nand42aa1d28x5               g160(.a(\b[20] ), .b(\a[21] ), .o1(new_n256));
  nor042aa1n04x5               g161(.a(\b[21] ), .b(\a[22] ), .o1(new_n257));
  nand42aa1d28x5               g162(.a(\b[21] ), .b(\a[22] ), .o1(new_n258));
  nano23aa1d15x5               g163(.a(new_n250), .b(new_n257), .c(new_n258), .d(new_n256), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nano32aa1n02x4               g165(.a(new_n260), .b(new_n219), .c(new_n225), .d(new_n231), .out0(new_n261));
  aoai13aa1n02x7               g166(.a(new_n261), .b(new_n210), .c(new_n159), .d(new_n199), .o1(new_n262));
  oai022aa1n02x5               g167(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n263));
  aoi022aa1n02x5               g168(.a(new_n243), .b(new_n259), .c(new_n258), .d(new_n263), .o1(new_n264));
  inv040aa1n02x5               g169(.a(new_n264), .o1(new_n265));
  nor042aa1n06x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  tech160nm_finand02aa1n03p5x5 g171(.a(\b[22] ), .b(\a[23] ), .o1(new_n267));
  norb02aa1n12x5               g172(.a(new_n267), .b(new_n266), .out0(new_n268));
  aoai13aa1n04x5               g173(.a(new_n268), .b(new_n265), .c(new_n205), .d(new_n261), .o1(new_n269));
  aoi122aa1n02x5               g174(.a(new_n268), .b(new_n258), .c(new_n263), .d(new_n243), .e(new_n259), .o1(new_n270));
  aobi12aa1n02x7               g175(.a(new_n269), .b(new_n270), .c(new_n262), .out0(\s[23] ));
  tech160nm_fixorc02aa1n04x5   g176(.a(\a[24] ), .b(\b[23] ), .out0(new_n272));
  inv040aa1d32x5               g177(.a(\a[24] ), .o1(new_n273));
  inv040aa1d30x5               g178(.a(\b[23] ), .o1(new_n274));
  nand22aa1n02x5               g179(.a(new_n274), .b(new_n273), .o1(new_n275));
  nanp02aa1n02x5               g180(.a(\b[23] ), .b(\a[24] ), .o1(new_n276));
  aoi012aa1n02x5               g181(.a(new_n266), .b(new_n275), .c(new_n276), .o1(new_n277));
  inv020aa1n02x5               g182(.a(new_n266), .o1(new_n278));
  inv000aa1d42x5               g183(.a(new_n268), .o1(new_n279));
  aoai13aa1n03x5               g184(.a(new_n278), .b(new_n279), .c(new_n262), .d(new_n264), .o1(new_n280));
  aoi022aa1n02x5               g185(.a(new_n280), .b(new_n272), .c(new_n269), .d(new_n277), .o1(\s[24] ));
  nand23aa1d12x5               g186(.a(new_n259), .b(new_n268), .c(new_n272), .o1(new_n282));
  nano32aa1n02x4               g187(.a(new_n282), .b(new_n219), .c(new_n225), .d(new_n231), .out0(new_n283));
  aoai13aa1n02x5               g188(.a(new_n283), .b(new_n210), .c(new_n159), .d(new_n199), .o1(new_n284));
  nand43aa1n02x5               g189(.a(new_n275), .b(new_n267), .c(new_n276), .o1(new_n285));
  oai112aa1n04x5               g190(.a(new_n278), .b(new_n258), .c(new_n257), .d(new_n250), .o1(new_n286));
  aob012aa1n02x5               g191(.a(new_n275), .b(new_n266), .c(new_n276), .out0(new_n287));
  oab012aa1n06x5               g192(.a(new_n287), .b(new_n286), .c(new_n285), .out0(new_n288));
  aoai13aa1n12x5               g193(.a(new_n288), .b(new_n282), .c(new_n241), .d(new_n242), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[25] ), .b(\b[24] ), .out0(new_n290));
  aoai13aa1n04x5               g195(.a(new_n290), .b(new_n289), .c(new_n205), .d(new_n283), .o1(new_n291));
  orn002aa1n02x5               g196(.a(new_n286), .b(new_n285), .o(new_n292));
  nona22aa1n02x4               g197(.a(new_n292), .b(new_n287), .c(new_n290), .out0(new_n293));
  aoib12aa1n02x5               g198(.a(new_n293), .b(new_n243), .c(new_n282), .out0(new_n294));
  aobi12aa1n03x7               g199(.a(new_n291), .b(new_n294), .c(new_n284), .out0(\s[25] ));
  xorc02aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .out0(new_n296));
  nor042aa1n03x5               g201(.a(\b[24] ), .b(\a[25] ), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n296), .b(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n289), .o1(new_n299));
  inv000aa1d42x5               g204(.a(new_n297), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n290), .o1(new_n301));
  aoai13aa1n03x5               g206(.a(new_n300), .b(new_n301), .c(new_n284), .d(new_n299), .o1(new_n302));
  aoi022aa1n02x5               g207(.a(new_n302), .b(new_n296), .c(new_n291), .d(new_n298), .o1(\s[26] ));
  and002aa1n06x5               g208(.a(new_n296), .b(new_n290), .o(new_n304));
  nano22aa1n03x7               g209(.a(new_n282), .b(new_n238), .c(new_n304), .out0(new_n305));
  aoai13aa1n06x5               g210(.a(new_n305), .b(new_n210), .c(new_n159), .d(new_n199), .o1(new_n306));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  and002aa1n12x5               g212(.a(\b[26] ), .b(\a[27] ), .o(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n307), .o1(new_n309));
  inv000aa1n03x5               g214(.a(new_n307), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[26] ), .b(\b[25] ), .c(new_n300), .carry(new_n311));
  oaib12aa1n02x5               g216(.a(new_n311), .b(new_n308), .c(new_n310), .out0(new_n312));
  aoi012aa1n02x5               g217(.a(new_n312), .b(new_n289), .c(new_n304), .o1(new_n313));
  aobi12aa1n18x5               g218(.a(new_n311), .b(new_n289), .c(new_n304), .out0(new_n314));
  nanp02aa1n02x5               g219(.a(new_n306), .b(new_n314), .o1(new_n315));
  aoi022aa1n02x5               g220(.a(new_n315), .b(new_n309), .c(new_n306), .d(new_n313), .o1(\s[27] ));
  inv000aa1d42x5               g221(.a(new_n308), .o1(new_n317));
  inv040aa1n08x5               g222(.a(new_n314), .o1(new_n318));
  aoai13aa1n06x5               g223(.a(new_n317), .b(new_n318), .c(new_n205), .d(new_n305), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n310), .b(new_n308), .c(new_n306), .d(new_n314), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[28] ), .b(\b[27] ), .out0(new_n321));
  norp02aa1n02x5               g226(.a(new_n321), .b(new_n307), .o1(new_n322));
  aoi022aa1n03x5               g227(.a(new_n320), .b(new_n321), .c(new_n319), .d(new_n322), .o1(\s[28] ));
  and002aa1n06x5               g228(.a(new_n321), .b(new_n309), .o(new_n324));
  aoai13aa1n06x5               g229(.a(new_n324), .b(new_n318), .c(new_n205), .d(new_n305), .o1(new_n325));
  inv040aa1n03x5               g230(.a(new_n324), .o1(new_n326));
  oao003aa1n02x5               g231(.a(\a[28] ), .b(\b[27] ), .c(new_n310), .carry(new_n327));
  aoai13aa1n02x7               g232(.a(new_n327), .b(new_n326), .c(new_n306), .d(new_n314), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[29] ), .b(\b[28] ), .out0(new_n329));
  norb02aa1n02x5               g234(.a(new_n327), .b(new_n329), .out0(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n325), .d(new_n330), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g237(.a(new_n321), .b(new_n329), .c(new_n309), .o(new_n333));
  aoai13aa1n06x5               g238(.a(new_n333), .b(new_n318), .c(new_n205), .d(new_n305), .o1(new_n334));
  inv000aa1d42x5               g239(.a(new_n333), .o1(new_n335));
  inv000aa1d42x5               g240(.a(\b[28] ), .o1(new_n336));
  inv000aa1d42x5               g241(.a(\a[29] ), .o1(new_n337));
  oaib12aa1n02x5               g242(.a(new_n327), .b(\b[28] ), .c(new_n337), .out0(new_n338));
  oaib12aa1n02x5               g243(.a(new_n338), .b(new_n336), .c(\a[29] ), .out0(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n335), .c(new_n306), .d(new_n314), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .out0(new_n341));
  oaoi13aa1n02x5               g246(.a(new_n341), .b(new_n338), .c(new_n337), .d(new_n336), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n341), .c(new_n334), .d(new_n342), .o1(\s[30] ));
  nano22aa1n12x5               g248(.a(new_n326), .b(new_n329), .c(new_n341), .out0(new_n344));
  aoai13aa1n06x5               g249(.a(new_n344), .b(new_n318), .c(new_n205), .d(new_n305), .o1(new_n345));
  aoi022aa1n02x5               g250(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n346));
  norb02aa1n02x5               g251(.a(\b[30] ), .b(\a[31] ), .out0(new_n347));
  obai22aa1n02x7               g252(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n348));
  aoi112aa1n02x5               g253(.a(new_n348), .b(new_n347), .c(new_n338), .d(new_n346), .o1(new_n349));
  inv040aa1n02x5               g254(.a(new_n344), .o1(new_n350));
  norp02aa1n02x5               g255(.a(\b[29] ), .b(\a[30] ), .o1(new_n351));
  aoi012aa1n02x5               g256(.a(new_n351), .b(new_n338), .c(new_n346), .o1(new_n352));
  aoai13aa1n03x5               g257(.a(new_n352), .b(new_n350), .c(new_n306), .d(new_n314), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[31] ), .b(\b[30] ), .out0(new_n354));
  aoi022aa1n02x7               g259(.a(new_n353), .b(new_n354), .c(new_n349), .d(new_n345), .o1(\s[31] ));
  norb02aa1n02x5               g260(.a(new_n103), .b(new_n102), .out0(new_n356));
  xobna2aa1n03x5               g261(.a(new_n356), .b(new_n150), .c(new_n98), .out0(\s[3] ));
  aoai13aa1n02x5               g262(.a(new_n356), .b(new_n100), .c(\a[2] ), .d(\b[1] ), .o1(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n101), .b(new_n358), .c(new_n103), .out0(\s[4] ));
  xobna2aa1n03x5               g264(.a(new_n154), .b(new_n152), .c(new_n106), .out0(\s[5] ));
  aoai13aa1n02x5               g265(.a(new_n153), .b(new_n110), .c(new_n107), .d(new_n111), .o1(new_n361));
  aoi112aa1n02x5               g266(.a(new_n110), .b(new_n153), .c(new_n107), .d(new_n111), .o1(new_n362));
  norb02aa1n02x5               g267(.a(new_n361), .b(new_n362), .out0(\s[6] ));
  inv000aa1d42x5               g268(.a(new_n108), .o1(new_n364));
  xnbna2aa1n03x5               g269(.a(new_n116), .b(new_n361), .c(new_n364), .out0(\s[7] ));
  aob012aa1n02x5               g270(.a(new_n116), .b(new_n361), .c(new_n364), .out0(new_n366));
  oai012aa1n02x5               g271(.a(new_n366), .b(\b[6] ), .c(\a[7] ), .o1(new_n367));
  norp02aa1n02x5               g272(.a(new_n113), .b(new_n114), .o1(new_n368));
  aoi022aa1n02x5               g273(.a(new_n367), .b(new_n113), .c(new_n366), .d(new_n368), .o1(\s[8] ));
  nanp02aa1n02x5               g274(.a(new_n107), .b(new_n117), .o1(new_n370));
  aoi113aa1n02x5               g275(.a(new_n157), .b(new_n127), .c(new_n118), .d(new_n113), .e(new_n119), .o1(new_n371));
  aoi022aa1n02x5               g276(.a(new_n159), .b(new_n127), .c(new_n370), .d(new_n371), .o1(\s[9] ));
endmodule


