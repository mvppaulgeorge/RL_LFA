// Benchmark "adder" written by ABC on Wed Jul 17 15:56:45 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n139,
    new_n140, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n148, new_n149, new_n150, new_n151, new_n152, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n180, new_n181, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n208, new_n209,
    new_n210, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n250, new_n251, new_n252, new_n253, new_n254, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n325, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n351, new_n353,
    new_n355, new_n356, new_n357, new_n358, new_n360, new_n361, new_n363,
    new_n365, new_n366;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n12x5               g001(.a(\b[9] ), .b(\a[10] ), .o1(new_n97));
  nand42aa1n08x5               g002(.a(\b[9] ), .b(\a[10] ), .o1(new_n98));
  norb02aa1n03x5               g003(.a(new_n98), .b(new_n97), .out0(new_n99));
  nor042aa1n12x5               g004(.a(\b[8] ), .b(\a[9] ), .o1(new_n100));
  inv000aa1d42x5               g005(.a(new_n100), .o1(new_n101));
  inv040aa1d32x5               g006(.a(\a[2] ), .o1(new_n102));
  inv000aa1d42x5               g007(.a(\b[1] ), .o1(new_n103));
  nand42aa1n02x5               g008(.a(new_n103), .b(new_n102), .o1(new_n104));
  nand42aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nand22aa1n12x5               g010(.a(\b[0] ), .b(\a[1] ), .o1(new_n106));
  aob012aa1n06x5               g011(.a(new_n104), .b(new_n105), .c(new_n106), .out0(new_n107));
  nor042aa1n04x5               g012(.a(\b[3] ), .b(\a[4] ), .o1(new_n108));
  nand42aa1n16x5               g013(.a(\b[3] ), .b(\a[4] ), .o1(new_n109));
  norb02aa1n06x5               g014(.a(new_n109), .b(new_n108), .out0(new_n110));
  nor042aa1n04x5               g015(.a(\b[2] ), .b(\a[3] ), .o1(new_n111));
  nanp02aa1n04x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  norb02aa1n06x5               g017(.a(new_n112), .b(new_n111), .out0(new_n113));
  nanp03aa1n09x5               g018(.a(new_n107), .b(new_n110), .c(new_n113), .o1(new_n114));
  aoi012aa1n12x5               g019(.a(new_n108), .b(new_n111), .c(new_n109), .o1(new_n115));
  nor042aa1n06x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nand22aa1n12x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand22aa1n06x5               g022(.a(\b[4] ), .b(\a[5] ), .o1(new_n118));
  nor002aa1d32x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nano23aa1n03x7               g024(.a(new_n119), .b(new_n116), .c(new_n117), .d(new_n118), .out0(new_n120));
  nor002aa1d32x5               g025(.a(\b[7] ), .b(\a[8] ), .o1(new_n121));
  nand22aa1n12x5               g026(.a(\b[7] ), .b(\a[8] ), .o1(new_n122));
  nanb02aa1n02x5               g027(.a(new_n121), .b(new_n122), .out0(new_n123));
  nand42aa1d28x5               g028(.a(\b[6] ), .b(\a[7] ), .o1(new_n124));
  nor042aa1n12x5               g029(.a(\b[6] ), .b(\a[7] ), .o1(new_n125));
  nanb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  nona22aa1n03x5               g031(.a(new_n120), .b(new_n123), .c(new_n126), .out0(new_n127));
  norb02aa1n12x5               g032(.a(new_n122), .b(new_n121), .out0(new_n128));
  norb02aa1n02x7               g033(.a(new_n124), .b(new_n125), .out0(new_n129));
  nand42aa1n04x5               g034(.a(new_n119), .b(new_n117), .o1(new_n130));
  oai012aa1n06x5               g035(.a(new_n130), .b(\b[5] ), .c(\a[6] ), .o1(new_n131));
  tech160nm_fiaoi012aa1n03p5x5 g036(.a(new_n121), .b(new_n125), .c(new_n122), .o1(new_n132));
  inv000aa1n02x5               g037(.a(new_n132), .o1(new_n133));
  aoi013aa1n06x4               g038(.a(new_n133), .b(new_n131), .c(new_n129), .d(new_n128), .o1(new_n134));
  aoai13aa1n12x5               g039(.a(new_n134), .b(new_n127), .c(new_n114), .d(new_n115), .o1(new_n135));
  tech160nm_fixorc02aa1n03p5x5 g040(.a(\a[9] ), .b(\b[8] ), .out0(new_n136));
  nanp02aa1n06x5               g041(.a(new_n135), .b(new_n136), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n99), .b(new_n137), .c(new_n101), .out0(\s[10] ));
  nor002aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  tech160nm_finand02aa1n03p5x5 g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n03x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  inv000aa1d42x5               g046(.a(new_n97), .o1(new_n142));
  inv000aa1d42x5               g047(.a(new_n98), .o1(new_n143));
  aoai13aa1n06x5               g048(.a(new_n142), .b(new_n143), .c(new_n137), .d(new_n101), .o1(new_n144));
  nand22aa1n03x5               g049(.a(new_n144), .b(new_n141), .o1(new_n145));
  aoi013aa1n02x4               g050(.a(new_n143), .b(new_n137), .c(new_n101), .d(new_n142), .o1(new_n146));
  oa0012aa1n03x5               g051(.a(new_n145), .b(new_n146), .c(new_n141), .o(\s[11] ));
  nor042aa1n02x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand42aa1n08x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  aoai13aa1n03x5               g055(.a(new_n150), .b(new_n139), .c(new_n144), .d(new_n140), .o1(new_n151));
  nona22aa1n03x5               g056(.a(new_n145), .b(new_n150), .c(new_n139), .out0(new_n152));
  nanp02aa1n03x5               g057(.a(new_n152), .b(new_n151), .o1(\s[12] ));
  nano32aa1n02x4               g058(.a(new_n150), .b(new_n136), .c(new_n141), .d(new_n99), .out0(new_n154));
  nanp02aa1n06x5               g059(.a(new_n135), .b(new_n154), .o1(new_n155));
  inv000aa1n02x5               g060(.a(new_n149), .o1(new_n156));
  tech160nm_fioai012aa1n03p5x5 g061(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .o1(new_n157));
  oab012aa1n06x5               g062(.a(new_n139), .b(new_n97), .c(new_n100), .out0(new_n158));
  nona32aa1n09x5               g063(.a(new_n158), .b(new_n157), .c(new_n156), .d(new_n143), .out0(new_n159));
  aoi012aa1n12x5               g064(.a(new_n148), .b(new_n139), .c(new_n149), .o1(new_n160));
  tech160nm_finand02aa1n05x5   g065(.a(new_n159), .b(new_n160), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nanp02aa1n06x5               g067(.a(new_n155), .b(new_n162), .o1(new_n163));
  nor042aa1n04x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand42aa1d28x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nano22aa1n02x4               g071(.a(new_n166), .b(new_n159), .c(new_n160), .out0(new_n167));
  aoi022aa1n02x5               g072(.a(new_n163), .b(new_n166), .c(new_n155), .d(new_n167), .o1(\s[13] ));
  tech160nm_fiaoi012aa1n05x5   g073(.a(new_n164), .b(new_n163), .c(new_n165), .o1(new_n169));
  xnrb03aa1n03x5               g074(.a(new_n169), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g075(.a(\b[13] ), .b(\a[14] ), .o1(new_n171));
  nand42aa1n20x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nano23aa1d15x5               g077(.a(new_n164), .b(new_n171), .c(new_n172), .d(new_n165), .out0(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n161), .c(new_n135), .d(new_n154), .o1(new_n174));
  oa0012aa1n02x5               g079(.a(new_n172), .b(new_n171), .c(new_n164), .o(new_n175));
  nanb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  nor002aa1n04x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  nand22aa1n03x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  norb02aa1n06x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  oaih22aa1d12x5               g084(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n180));
  aboi22aa1n03x5               g085(.a(new_n177), .b(new_n178), .c(new_n180), .d(new_n172), .out0(new_n181));
  aoi022aa1n02x5               g086(.a(new_n176), .b(new_n179), .c(new_n174), .d(new_n181), .o1(\s[15] ));
  xorc02aa1n12x5               g087(.a(\a[16] ), .b(\b[15] ), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n183), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n184), .b(new_n177), .c(new_n176), .d(new_n178), .o1(new_n185));
  aoai13aa1n04x5               g090(.a(new_n179), .b(new_n175), .c(new_n163), .d(new_n173), .o1(new_n186));
  nona22aa1n02x4               g091(.a(new_n186), .b(new_n184), .c(new_n177), .out0(new_n187));
  nanp02aa1n02x5               g092(.a(new_n185), .b(new_n187), .o1(\s[16] ));
  nano23aa1n02x5               g093(.a(new_n139), .b(new_n148), .c(new_n149), .d(new_n140), .out0(new_n189));
  nand23aa1n09x5               g094(.a(new_n173), .b(new_n179), .c(new_n183), .o1(new_n190));
  nano32aa1n03x7               g095(.a(new_n190), .b(new_n189), .c(new_n136), .d(new_n99), .out0(new_n191));
  nand02aa1d06x5               g096(.a(new_n135), .b(new_n191), .o1(new_n192));
  aoi012aa1n12x5               g097(.a(new_n190), .b(new_n159), .c(new_n160), .o1(new_n193));
  inv000aa1d42x5               g098(.a(\a[16] ), .o1(new_n194));
  inv000aa1d42x5               g099(.a(\b[15] ), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n195), .b(new_n194), .o1(new_n196));
  nanp02aa1n02x5               g101(.a(\b[15] ), .b(\a[16] ), .o1(new_n197));
  nanp03aa1n02x5               g102(.a(new_n196), .b(new_n178), .c(new_n197), .o1(new_n198));
  oai112aa1n02x5               g103(.a(new_n180), .b(new_n172), .c(\b[14] ), .d(\a[15] ), .o1(new_n199));
  tech160nm_fioaoi03aa1n03p5x5 g104(.a(new_n194), .b(new_n195), .c(new_n177), .o1(new_n200));
  oa0012aa1n02x5               g105(.a(new_n200), .b(new_n199), .c(new_n198), .o(new_n201));
  norb02aa1n06x5               g106(.a(new_n201), .b(new_n193), .out0(new_n202));
  nand02aa1d06x5               g107(.a(new_n192), .b(new_n202), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .out0(new_n204));
  orn002aa1n02x5               g109(.a(new_n199), .b(new_n198), .o(new_n205));
  nano23aa1n02x4               g110(.a(new_n193), .b(new_n204), .c(new_n205), .d(new_n200), .out0(new_n206));
  aoi022aa1n02x5               g111(.a(new_n203), .b(new_n204), .c(new_n192), .d(new_n206), .o1(\s[17] ));
  inv040aa1d32x5               g112(.a(\a[18] ), .o1(new_n208));
  norp02aa1n06x5               g113(.a(\b[16] ), .b(\a[17] ), .o1(new_n209));
  tech160nm_fiaoi012aa1n05x5   g114(.a(new_n209), .b(new_n203), .c(new_n204), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[17] ), .c(new_n208), .out0(\s[18] ));
  aoai13aa1n06x5               g116(.a(new_n201), .b(new_n190), .c(new_n159), .d(new_n160), .o1(new_n212));
  inv000aa1d42x5               g117(.a(\a[17] ), .o1(new_n213));
  xroi22aa1d06x4               g118(.a(new_n213), .b(\b[16] ), .c(new_n208), .d(\b[17] ), .out0(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n212), .c(new_n135), .d(new_n191), .o1(new_n215));
  nand02aa1n03x5               g120(.a(\b[17] ), .b(\a[18] ), .o1(new_n216));
  nor042aa1n02x5               g121(.a(\b[17] ), .b(\a[18] ), .o1(new_n217));
  norp02aa1n04x5               g122(.a(new_n217), .b(new_n209), .o1(new_n218));
  norb02aa1n02x5               g123(.a(new_n216), .b(new_n218), .out0(new_n219));
  inv000aa1n02x5               g124(.a(new_n219), .o1(new_n220));
  nor042aa1n09x5               g125(.a(\b[18] ), .b(\a[19] ), .o1(new_n221));
  nand42aa1n06x5               g126(.a(\b[18] ), .b(\a[19] ), .o1(new_n222));
  norb02aa1d21x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n215), .c(new_n220), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp02aa1n06x5               g130(.a(new_n215), .b(new_n220), .o1(new_n226));
  nor042aa1n09x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand42aa1n16x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nanb02aa1n06x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  aoai13aa1n02x5               g134(.a(new_n229), .b(new_n221), .c(new_n226), .d(new_n222), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(new_n226), .b(new_n223), .o1(new_n231));
  nona22aa1n02x4               g136(.a(new_n231), .b(new_n229), .c(new_n221), .out0(new_n232));
  nanp02aa1n03x5               g137(.a(new_n232), .b(new_n230), .o1(\s[20] ));
  nanb03aa1d18x5               g138(.a(new_n229), .b(new_n214), .c(new_n223), .out0(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n212), .c(new_n135), .d(new_n191), .o1(new_n236));
  nanb03aa1n02x5               g141(.a(new_n227), .b(new_n228), .c(new_n222), .out0(new_n237));
  orn002aa1n02x5               g142(.a(\a[19] ), .b(\b[18] ), .o(new_n238));
  oai112aa1n02x5               g143(.a(new_n238), .b(new_n216), .c(new_n217), .d(new_n209), .o1(new_n239));
  aoi012aa1d18x5               g144(.a(new_n227), .b(new_n221), .c(new_n228), .o1(new_n240));
  oai012aa1n06x5               g145(.a(new_n240), .b(new_n239), .c(new_n237), .o1(new_n241));
  nanb02aa1n06x5               g146(.a(new_n241), .b(new_n236), .out0(new_n242));
  xorc02aa1n12x5               g147(.a(\a[21] ), .b(\b[20] ), .out0(new_n243));
  nano22aa1n03x7               g148(.a(new_n227), .b(new_n222), .c(new_n228), .out0(new_n244));
  oaih12aa1n02x5               g149(.a(new_n216), .b(\b[18] ), .c(\a[19] ), .o1(new_n245));
  oab012aa1n03x5               g150(.a(new_n245), .b(new_n209), .c(new_n217), .out0(new_n246));
  inv000aa1d42x5               g151(.a(new_n240), .o1(new_n247));
  aoi112aa1n02x5               g152(.a(new_n247), .b(new_n243), .c(new_n246), .d(new_n244), .o1(new_n248));
  aoi022aa1n02x5               g153(.a(new_n242), .b(new_n243), .c(new_n236), .d(new_n248), .o1(\s[21] ));
  nor042aa1n04x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  xnrc02aa1n12x5               g155(.a(\b[21] ), .b(\a[22] ), .out0(new_n251));
  aoai13aa1n02x5               g156(.a(new_n251), .b(new_n250), .c(new_n242), .d(new_n243), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n243), .b(new_n241), .c(new_n203), .d(new_n235), .o1(new_n253));
  nona22aa1n03x5               g158(.a(new_n253), .b(new_n251), .c(new_n250), .out0(new_n254));
  nanp02aa1n03x5               g159(.a(new_n252), .b(new_n254), .o1(\s[22] ));
  tech160nm_fixnrc02aa1n02p5x5 g160(.a(\b[20] ), .b(\a[21] ), .out0(new_n256));
  nona32aa1n02x4               g161(.a(new_n203), .b(new_n251), .c(new_n256), .d(new_n234), .out0(new_n257));
  nor042aa1n03x5               g162(.a(new_n251), .b(new_n256), .o1(new_n258));
  nanb02aa1n03x5               g163(.a(new_n234), .b(new_n258), .out0(new_n259));
  nona22aa1n09x5               g164(.a(new_n244), .b(new_n218), .c(new_n245), .out0(new_n260));
  nanb02aa1n06x5               g165(.a(new_n251), .b(new_n243), .out0(new_n261));
  inv000aa1d42x5               g166(.a(\a[22] ), .o1(new_n262));
  inv000aa1d42x5               g167(.a(\b[21] ), .o1(new_n263));
  oao003aa1n12x5               g168(.a(new_n262), .b(new_n263), .c(new_n250), .carry(new_n264));
  inv040aa1n03x5               g169(.a(new_n264), .o1(new_n265));
  aoai13aa1n12x5               g170(.a(new_n265), .b(new_n261), .c(new_n260), .d(new_n240), .o1(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  aoai13aa1n04x5               g172(.a(new_n267), .b(new_n259), .c(new_n192), .d(new_n202), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[23] ), .b(\b[22] ), .out0(new_n269));
  aoi112aa1n02x5               g174(.a(new_n269), .b(new_n264), .c(new_n241), .d(new_n258), .o1(new_n270));
  aoi022aa1n02x5               g175(.a(new_n270), .b(new_n257), .c(new_n268), .d(new_n269), .o1(\s[23] ));
  nor002aa1n02x5               g176(.a(\b[22] ), .b(\a[23] ), .o1(new_n272));
  xnrc02aa1n12x5               g177(.a(\b[23] ), .b(\a[24] ), .out0(new_n273));
  aoai13aa1n03x5               g178(.a(new_n273), .b(new_n272), .c(new_n268), .d(new_n269), .o1(new_n274));
  nanp02aa1n03x5               g179(.a(new_n268), .b(new_n269), .o1(new_n275));
  nona22aa1n03x5               g180(.a(new_n275), .b(new_n273), .c(new_n272), .out0(new_n276));
  nanp02aa1n03x5               g181(.a(new_n276), .b(new_n274), .o1(\s[24] ));
  norb02aa1n06x5               g182(.a(new_n269), .b(new_n273), .out0(new_n278));
  nano22aa1n03x7               g183(.a(new_n234), .b(new_n258), .c(new_n278), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n212), .c(new_n135), .d(new_n191), .o1(new_n280));
  aoai13aa1n04x5               g185(.a(new_n258), .b(new_n247), .c(new_n246), .d(new_n244), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n278), .o1(new_n282));
  inv000aa1d42x5               g187(.a(\a[24] ), .o1(new_n283));
  inv000aa1d42x5               g188(.a(\b[23] ), .o1(new_n284));
  oaoi03aa1n12x5               g189(.a(new_n283), .b(new_n284), .c(new_n272), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n285), .b(new_n282), .c(new_n281), .d(new_n265), .o1(new_n286));
  nanb02aa1n06x5               g191(.a(new_n286), .b(new_n280), .out0(new_n287));
  xorc02aa1n12x5               g192(.a(\a[25] ), .b(\b[24] ), .out0(new_n288));
  inv000aa1d42x5               g193(.a(new_n285), .o1(new_n289));
  aoi112aa1n02x5               g194(.a(new_n288), .b(new_n289), .c(new_n266), .d(new_n278), .o1(new_n290));
  aoi022aa1n02x5               g195(.a(new_n287), .b(new_n288), .c(new_n280), .d(new_n290), .o1(\s[25] ));
  norp02aa1n02x5               g196(.a(\b[24] ), .b(\a[25] ), .o1(new_n292));
  xnrc02aa1n12x5               g197(.a(\b[25] ), .b(\a[26] ), .out0(new_n293));
  aoai13aa1n02x5               g198(.a(new_n293), .b(new_n292), .c(new_n287), .d(new_n288), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n288), .b(new_n286), .c(new_n203), .d(new_n279), .o1(new_n295));
  nona22aa1n03x5               g200(.a(new_n295), .b(new_n293), .c(new_n292), .out0(new_n296));
  nanp02aa1n03x5               g201(.a(new_n294), .b(new_n296), .o1(\s[26] ));
  norb02aa1n06x5               g202(.a(new_n288), .b(new_n293), .out0(new_n298));
  nano32aa1d12x5               g203(.a(new_n234), .b(new_n298), .c(new_n258), .d(new_n278), .out0(new_n299));
  aoai13aa1n06x5               g204(.a(new_n299), .b(new_n212), .c(new_n135), .d(new_n191), .o1(new_n300));
  orn002aa1n02x5               g205(.a(\a[25] ), .b(\b[24] ), .o(new_n301));
  oao003aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n301), .carry(new_n302));
  aobi12aa1n03x5               g207(.a(new_n302), .b(new_n286), .c(new_n298), .out0(new_n303));
  tech160nm_fixorc02aa1n05x5   g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  xnbna2aa1n03x5               g209(.a(new_n304), .b(new_n303), .c(new_n300), .out0(\s[27] ));
  aoai13aa1n04x5               g210(.a(new_n298), .b(new_n289), .c(new_n266), .d(new_n278), .o1(new_n306));
  nand43aa1n03x5               g211(.a(new_n300), .b(new_n306), .c(new_n302), .o1(new_n307));
  norp02aa1n02x5               g212(.a(\b[26] ), .b(\a[27] ), .o1(new_n308));
  norp02aa1n02x5               g213(.a(\b[27] ), .b(\a[28] ), .o1(new_n309));
  nanp02aa1n02x5               g214(.a(\b[27] ), .b(\a[28] ), .o1(new_n310));
  nanb02aa1n02x5               g215(.a(new_n309), .b(new_n310), .out0(new_n311));
  aoai13aa1n02x7               g216(.a(new_n311), .b(new_n308), .c(new_n307), .d(new_n304), .o1(new_n312));
  aoai13aa1n03x5               g217(.a(new_n278), .b(new_n264), .c(new_n241), .d(new_n258), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n298), .o1(new_n314));
  aoai13aa1n06x5               g219(.a(new_n302), .b(new_n314), .c(new_n313), .d(new_n285), .o1(new_n315));
  aoai13aa1n02x5               g220(.a(new_n304), .b(new_n315), .c(new_n203), .d(new_n299), .o1(new_n316));
  nona22aa1n03x5               g221(.a(new_n316), .b(new_n311), .c(new_n308), .out0(new_n317));
  nanp02aa1n03x5               g222(.a(new_n312), .b(new_n317), .o1(\s[28] ));
  norb02aa1n09x5               g223(.a(new_n304), .b(new_n311), .out0(new_n319));
  aoai13aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n203), .d(new_n299), .o1(new_n320));
  xorc02aa1n02x5               g225(.a(\a[29] ), .b(\b[28] ), .out0(new_n321));
  oai012aa1n02x5               g226(.a(new_n310), .b(new_n309), .c(new_n308), .o1(new_n322));
  norb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  inv000aa1d42x5               g228(.a(new_n319), .o1(new_n324));
  aoai13aa1n02x7               g229(.a(new_n322), .b(new_n324), .c(new_n303), .d(new_n300), .o1(new_n325));
  aoi022aa1n03x5               g230(.a(new_n325), .b(new_n321), .c(new_n320), .d(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g231(.a(new_n106), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nanb03aa1n02x5               g232(.a(new_n311), .b(new_n321), .c(new_n304), .out0(new_n328));
  nanb02aa1n03x5               g233(.a(new_n328), .b(new_n307), .out0(new_n329));
  inv000aa1d42x5               g234(.a(\b[28] ), .o1(new_n330));
  inv000aa1d42x5               g235(.a(\a[29] ), .o1(new_n331));
  oaib12aa1n02x5               g236(.a(new_n322), .b(\b[28] ), .c(new_n331), .out0(new_n332));
  oaib12aa1n02x5               g237(.a(new_n332), .b(new_n330), .c(\a[29] ), .out0(new_n333));
  aoai13aa1n02x7               g238(.a(new_n333), .b(new_n328), .c(new_n303), .d(new_n300), .o1(new_n334));
  xorc02aa1n02x5               g239(.a(\a[30] ), .b(\b[29] ), .out0(new_n335));
  oaoi13aa1n02x5               g240(.a(new_n335), .b(new_n332), .c(new_n331), .d(new_n330), .o1(new_n336));
  aoi022aa1n03x5               g241(.a(new_n334), .b(new_n335), .c(new_n329), .d(new_n336), .o1(\s[30] ));
  nano22aa1d33x5               g242(.a(new_n324), .b(new_n321), .c(new_n335), .out0(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n315), .c(new_n203), .d(new_n299), .o1(new_n339));
  aoi022aa1n02x5               g244(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n340));
  norb02aa1n02x5               g245(.a(\b[30] ), .b(\a[31] ), .out0(new_n341));
  obai22aa1n02x7               g246(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n342));
  aoi112aa1n02x5               g247(.a(new_n342), .b(new_n341), .c(new_n332), .d(new_n340), .o1(new_n343));
  inv000aa1d42x5               g248(.a(new_n338), .o1(new_n344));
  norp02aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n332), .c(new_n340), .o1(new_n346));
  aoai13aa1n02x7               g251(.a(new_n346), .b(new_n344), .c(new_n303), .d(new_n300), .o1(new_n347));
  xorc02aa1n02x5               g252(.a(\a[31] ), .b(\b[30] ), .out0(new_n348));
  aoi022aa1n02x7               g253(.a(new_n347), .b(new_n348), .c(new_n343), .d(new_n339), .o1(\s[31] ));
  xorb03aa1n02x5               g254(.a(new_n107), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oai012aa1n02x5               g255(.a(new_n112), .b(new_n107), .c(new_n111), .o1(new_n351));
  xnrc02aa1n02x5               g256(.a(new_n351), .b(new_n110), .out0(\s[4] ));
  norb02aa1n02x5               g257(.a(new_n118), .b(new_n119), .out0(new_n353));
  xnbna2aa1n03x5               g258(.a(new_n353), .b(new_n114), .c(new_n115), .out0(\s[5] ));
  norb02aa1n02x5               g259(.a(new_n117), .b(new_n116), .out0(new_n355));
  inv000aa1d42x5               g260(.a(new_n119), .o1(new_n356));
  nanp02aa1n02x5               g261(.a(new_n114), .b(new_n115), .o1(new_n357));
  nanp02aa1n02x5               g262(.a(new_n357), .b(new_n353), .o1(new_n358));
  xnbna2aa1n03x5               g263(.a(new_n355), .b(new_n358), .c(new_n356), .out0(\s[6] ));
  aobi12aa1n02x5               g264(.a(new_n355), .b(new_n358), .c(new_n356), .out0(new_n360));
  norp02aa1n02x5               g265(.a(new_n360), .b(new_n116), .o1(new_n361));
  xnrb03aa1n02x5               g266(.a(new_n361), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oai013aa1n02x4               g267(.a(new_n124), .b(new_n360), .c(new_n125), .d(new_n116), .o1(new_n363));
  xnrc02aa1n02x5               g268(.a(new_n363), .b(new_n128), .out0(\s[8] ));
  aoi012aa1n02x5               g269(.a(new_n127), .b(new_n114), .c(new_n115), .o1(new_n365));
  aoi113aa1n02x5               g270(.a(new_n136), .b(new_n133), .c(new_n131), .d(new_n129), .e(new_n128), .o1(new_n366));
  aboi22aa1n03x5               g271(.a(new_n365), .b(new_n366), .c(new_n135), .d(new_n136), .out0(\s[9] ));
endmodule


