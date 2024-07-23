// Benchmark "adder" written by ABC on Wed Jul 17 17:47:22 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n129, new_n130, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n164, new_n165, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n176, new_n177, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n262, new_n263, new_n264, new_n265,
    new_n267, new_n268, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n279, new_n280, new_n281,
    new_n282, new_n283, new_n285, new_n286, new_n287, new_n288, new_n289,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n307, new_n308, new_n309, new_n310, new_n311, new_n312,
    new_n313, new_n314, new_n317, new_n318, new_n319, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n328, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n341, new_n342, new_n344, new_n345, new_n347,
    new_n348, new_n350, new_n351, new_n352, new_n354, new_n355, new_n356,
    new_n358;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  orn002aa1n06x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nanp02aa1n04x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aob012aa1n03x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(new_n100));
  nanp02aa1n02x5               g005(.a(new_n100), .b(new_n98), .o1(new_n101));
  xorc02aa1n02x5               g006(.a(\a[3] ), .b(\b[2] ), .out0(new_n102));
  oa0022aa1n06x5               g007(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n103));
  aobi12aa1n02x5               g008(.a(new_n103), .b(new_n101), .c(new_n102), .out0(new_n104));
  nand42aa1n03x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[5] ), .b(\a[6] ), .o1(new_n106));
  nor022aa1n04x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nano22aa1n02x4               g012(.a(new_n107), .b(new_n105), .c(new_n106), .out0(new_n108));
  inv000aa1d42x5               g013(.a(\a[8] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[7] ), .o1(new_n110));
  aoi022aa1n06x5               g015(.a(new_n110), .b(new_n109), .c(\a[4] ), .d(\b[3] ), .o1(new_n111));
  nanp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor002aa1n06x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand22aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nor002aa1n16x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  nand03aa1n02x5               g021(.a(new_n116), .b(new_n108), .c(new_n111), .o1(new_n117));
  nano22aa1n03x7               g022(.a(new_n113), .b(new_n106), .c(new_n114), .out0(new_n118));
  oai022aa1n02x5               g023(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n119));
  xorc02aa1n03x5               g024(.a(\a[8] ), .b(\b[7] ), .out0(new_n120));
  oao003aa1n02x5               g025(.a(new_n109), .b(new_n110), .c(new_n113), .carry(new_n121));
  aoi013aa1n06x4               g026(.a(new_n121), .b(new_n118), .c(new_n120), .d(new_n119), .o1(new_n122));
  oai012aa1n06x5               g027(.a(new_n122), .b(new_n117), .c(new_n104), .o1(new_n123));
  nand42aa1n04x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nor002aa1d24x5               g029(.a(\b[9] ), .b(\a[10] ), .o1(new_n125));
  nand02aa1n10x5               g030(.a(\b[9] ), .b(\a[10] ), .o1(new_n126));
  norb02aa1n02x5               g031(.a(new_n126), .b(new_n125), .out0(new_n127));
  aoai13aa1n02x5               g032(.a(new_n127), .b(new_n97), .c(new_n123), .d(new_n124), .o1(new_n128));
  norb02aa1n02x5               g033(.a(new_n124), .b(new_n97), .out0(new_n129));
  aoi112aa1n02x5               g034(.a(new_n127), .b(new_n97), .c(new_n123), .d(new_n129), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n128), .b(new_n130), .out0(\s[10] ));
  nona23aa1d18x5               g036(.a(new_n126), .b(new_n124), .c(new_n97), .d(new_n125), .out0(new_n132));
  inv000aa1d42x5               g037(.a(new_n132), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(new_n123), .b(new_n133), .o1(new_n134));
  oai012aa1n02x5               g039(.a(new_n126), .b(new_n125), .c(new_n97), .o1(new_n135));
  xnrc02aa1n12x5               g040(.a(\b[10] ), .b(\a[11] ), .out0(new_n136));
  inv000aa1d42x5               g041(.a(new_n136), .o1(new_n137));
  xnbna2aa1n03x5               g042(.a(new_n137), .b(new_n134), .c(new_n135), .out0(\s[11] ));
  inv040aa1d32x5               g043(.a(\a[11] ), .o1(new_n139));
  inv000aa1d42x5               g044(.a(\b[10] ), .o1(new_n140));
  nanp02aa1n04x5               g045(.a(new_n140), .b(new_n139), .o1(new_n141));
  aob012aa1n02x5               g046(.a(new_n137), .b(new_n134), .c(new_n135), .out0(new_n142));
  nor042aa1n06x5               g047(.a(\b[11] ), .b(\a[12] ), .o1(new_n143));
  nand22aa1n06x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nanb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(new_n145));
  xobna2aa1n03x5               g050(.a(new_n145), .b(new_n142), .c(new_n141), .out0(\s[12] ));
  norp03aa1n02x5               g051(.a(new_n132), .b(new_n136), .c(new_n145), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n123), .b(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(\b[10] ), .b(\a[11] ), .o1(new_n149));
  nanb03aa1n06x5               g054(.a(new_n143), .b(new_n144), .c(new_n149), .out0(new_n150));
  oai112aa1n04x5               g055(.a(new_n126), .b(new_n141), .c(new_n125), .d(new_n97), .o1(new_n151));
  aoi013aa1n06x4               g056(.a(new_n143), .b(new_n144), .c(new_n139), .d(new_n140), .o1(new_n152));
  tech160nm_fioai012aa1n04x5   g057(.a(new_n152), .b(new_n151), .c(new_n150), .o1(new_n153));
  nanb02aa1n02x5               g058(.a(new_n153), .b(new_n148), .out0(new_n154));
  nor002aa1d32x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand02aa1d06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  oaib12aa1n02x5               g062(.a(new_n152), .b(new_n155), .c(new_n156), .out0(new_n158));
  oab012aa1n02x4               g063(.a(new_n158), .b(new_n151), .c(new_n150), .out0(new_n159));
  aoi022aa1n02x5               g064(.a(new_n154), .b(new_n157), .c(new_n148), .d(new_n159), .o1(\s[13] ));
  inv000aa1d42x5               g065(.a(new_n155), .o1(new_n161));
  aoai13aa1n02x5               g066(.a(new_n157), .b(new_n153), .c(new_n123), .d(new_n147), .o1(new_n162));
  nor002aa1d32x5               g067(.a(\b[13] ), .b(\a[14] ), .o1(new_n163));
  nanp02aa1n09x5               g068(.a(\b[13] ), .b(\a[14] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  xnbna2aa1n03x5               g070(.a(new_n165), .b(new_n162), .c(new_n161), .out0(\s[14] ));
  nona23aa1d18x5               g071(.a(new_n164), .b(new_n156), .c(new_n155), .d(new_n163), .out0(new_n167));
  inv000aa1d42x5               g072(.a(new_n167), .o1(new_n168));
  aoai13aa1n03x5               g073(.a(new_n168), .b(new_n153), .c(new_n123), .d(new_n147), .o1(new_n169));
  oaoi03aa1n02x5               g074(.a(\a[14] ), .b(\b[13] ), .c(new_n161), .o1(new_n170));
  inv000aa1d42x5               g075(.a(new_n170), .o1(new_n171));
  nor002aa1n16x5               g076(.a(\b[14] ), .b(\a[15] ), .o1(new_n172));
  nand22aa1n03x5               g077(.a(\b[14] ), .b(\a[15] ), .o1(new_n173));
  norb02aa1d27x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n169), .c(new_n171), .out0(\s[15] ));
  aob012aa1n03x5               g080(.a(new_n174), .b(new_n169), .c(new_n171), .out0(new_n176));
  nor042aa1n03x5               g081(.a(\b[15] ), .b(\a[16] ), .o1(new_n177));
  nand22aa1n03x5               g082(.a(\b[15] ), .b(\a[16] ), .o1(new_n178));
  norb02aa1n02x5               g083(.a(new_n178), .b(new_n177), .out0(new_n179));
  norp02aa1n02x5               g084(.a(new_n179), .b(new_n172), .o1(new_n180));
  inv020aa1n03x5               g085(.a(new_n172), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n174), .o1(new_n182));
  aoai13aa1n02x5               g087(.a(new_n181), .b(new_n182), .c(new_n169), .d(new_n171), .o1(new_n183));
  aoi022aa1n03x5               g088(.a(new_n183), .b(new_n179), .c(new_n176), .d(new_n180), .o1(\s[16] ));
  nona23aa1n06x5               g089(.a(new_n178), .b(new_n173), .c(new_n172), .d(new_n177), .out0(new_n185));
  nor042aa1n06x5               g090(.a(new_n185), .b(new_n167), .o1(new_n186));
  nona32aa1n09x5               g091(.a(new_n186), .b(new_n132), .c(new_n145), .d(new_n136), .out0(new_n187));
  nanb02aa1n02x5               g092(.a(new_n187), .b(new_n123), .out0(new_n188));
  xorc02aa1n02x5               g093(.a(\a[17] ), .b(\b[16] ), .out0(new_n189));
  nanb03aa1n03x5               g094(.a(new_n177), .b(new_n178), .c(new_n173), .out0(new_n190));
  oai112aa1n02x7               g095(.a(new_n181), .b(new_n164), .c(new_n163), .d(new_n155), .o1(new_n191));
  norp02aa1n02x5               g096(.a(new_n191), .b(new_n190), .o1(new_n192));
  tech160nm_fiaoi012aa1n03p5x5 g097(.a(new_n177), .b(new_n172), .c(new_n178), .o1(new_n193));
  nanb02aa1n02x5               g098(.a(new_n189), .b(new_n193), .out0(new_n194));
  aoi112aa1n02x5               g099(.a(new_n194), .b(new_n192), .c(new_n153), .d(new_n186), .o1(new_n195));
  orn002aa1n03x5               g100(.a(\a[3] ), .b(\b[2] ), .o(new_n196));
  nanp02aa1n02x5               g101(.a(\b[2] ), .b(\a[3] ), .o1(new_n197));
  nanp02aa1n06x5               g102(.a(new_n196), .b(new_n197), .o1(new_n198));
  aoai13aa1n04x5               g103(.a(new_n103), .b(new_n198), .c(new_n100), .d(new_n98), .o1(new_n199));
  aoi012aa1n02x7               g104(.a(new_n107), .b(\a[6] ), .c(\b[5] ), .o1(new_n200));
  nona23aa1n03x5               g105(.a(new_n114), .b(new_n112), .c(new_n115), .d(new_n113), .out0(new_n201));
  nano32aa1n03x7               g106(.a(new_n201), .b(new_n111), .c(new_n200), .d(new_n105), .out0(new_n202));
  tech160nm_finand02aa1n03p5x5 g107(.a(new_n202), .b(new_n199), .o1(new_n203));
  tech160nm_fioai012aa1n04x5   g108(.a(new_n193), .b(new_n191), .c(new_n190), .o1(new_n204));
  aoi012aa1n06x5               g109(.a(new_n204), .b(new_n153), .c(new_n186), .o1(new_n205));
  aoai13aa1n12x5               g110(.a(new_n205), .b(new_n187), .c(new_n203), .d(new_n122), .o1(new_n206));
  aoi022aa1n02x5               g111(.a(new_n206), .b(new_n189), .c(new_n188), .d(new_n195), .o1(\s[17] ));
  nor002aa1d24x5               g112(.a(\b[16] ), .b(\a[17] ), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  inv000aa1d42x5               g114(.a(\a[17] ), .o1(new_n210));
  oaib12aa1n02x5               g115(.a(new_n206), .b(new_n210), .c(\b[16] ), .out0(new_n211));
  xorc02aa1n02x5               g116(.a(\a[18] ), .b(\b[17] ), .out0(new_n212));
  xnbna2aa1n03x5               g117(.a(new_n212), .b(new_n211), .c(new_n209), .out0(\s[18] ));
  inv040aa1d32x5               g118(.a(\a[18] ), .o1(new_n214));
  xroi22aa1d06x4               g119(.a(new_n210), .b(\b[16] ), .c(new_n214), .d(\b[17] ), .out0(new_n215));
  nanp02aa1n06x5               g120(.a(new_n206), .b(new_n215), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[17] ), .o1(new_n217));
  oaoi03aa1n02x5               g122(.a(new_n214), .b(new_n217), .c(new_n208), .o1(new_n218));
  tech160nm_fixorc02aa1n05x5   g123(.a(\a[19] ), .b(\b[18] ), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n216), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aob012aa1n03x5               g126(.a(new_n219), .b(new_n216), .c(new_n218), .out0(new_n222));
  nor042aa1n03x5               g127(.a(\b[19] ), .b(\a[20] ), .o1(new_n223));
  and002aa1n06x5               g128(.a(\b[19] ), .b(\a[20] ), .o(new_n224));
  nor022aa1n04x5               g129(.a(new_n224), .b(new_n223), .o1(new_n225));
  norp02aa1n02x5               g130(.a(\b[18] ), .b(\a[19] ), .o1(new_n226));
  oab012aa1n02x4               g131(.a(new_n226), .b(new_n224), .c(new_n223), .out0(new_n227));
  aobi12aa1n02x5               g132(.a(new_n218), .b(new_n206), .c(new_n215), .out0(new_n228));
  oaoi03aa1n02x5               g133(.a(\a[19] ), .b(\b[18] ), .c(new_n228), .o1(new_n229));
  aoi022aa1n03x5               g134(.a(new_n229), .b(new_n225), .c(new_n222), .d(new_n227), .o1(\s[20] ));
  nand23aa1n04x5               g135(.a(new_n215), .b(new_n219), .c(new_n225), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoi012aa1n02x5               g137(.a(new_n208), .b(new_n214), .c(new_n217), .o1(new_n233));
  aoi112aa1n03x4               g138(.a(new_n224), .b(new_n223), .c(\a[19] ), .d(\b[18] ), .o1(new_n234));
  oai022aa1n02x5               g139(.a(new_n214), .b(new_n217), .c(\b[18] ), .d(\a[19] ), .o1(new_n235));
  nona22aa1n03x5               g140(.a(new_n234), .b(new_n235), .c(new_n233), .out0(new_n236));
  aoib12aa1n06x5               g141(.a(new_n223), .b(new_n226), .c(new_n224), .out0(new_n237));
  nanp02aa1n02x5               g142(.a(new_n236), .b(new_n237), .o1(new_n238));
  xorc02aa1n12x5               g143(.a(\a[21] ), .b(\b[20] ), .out0(new_n239));
  aoai13aa1n06x5               g144(.a(new_n239), .b(new_n238), .c(new_n206), .d(new_n232), .o1(new_n240));
  nano22aa1n02x4               g145(.a(new_n239), .b(new_n236), .c(new_n237), .out0(new_n241));
  aobi12aa1n02x5               g146(.a(new_n241), .b(new_n206), .c(new_n232), .out0(new_n242));
  norb02aa1n02x5               g147(.a(new_n240), .b(new_n242), .out0(\s[21] ));
  xorc02aa1n12x5               g148(.a(\a[22] ), .b(\b[21] ), .out0(new_n244));
  norp02aa1n02x5               g149(.a(\b[20] ), .b(\a[21] ), .o1(new_n245));
  norp02aa1n02x5               g150(.a(new_n244), .b(new_n245), .o1(new_n246));
  inv000aa1d42x5               g151(.a(\a[21] ), .o1(new_n247));
  oaib12aa1n06x5               g152(.a(new_n240), .b(\b[20] ), .c(new_n247), .out0(new_n248));
  aoi022aa1n02x7               g153(.a(new_n248), .b(new_n244), .c(new_n240), .d(new_n246), .o1(\s[22] ));
  nand02aa1d06x5               g154(.a(new_n244), .b(new_n239), .o1(new_n250));
  nano32aa1n02x4               g155(.a(new_n250), .b(new_n215), .c(new_n219), .d(new_n225), .out0(new_n251));
  oai022aa1n02x5               g156(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n252));
  aob012aa1n02x5               g157(.a(new_n252), .b(\b[21] ), .c(\a[22] ), .out0(new_n253));
  aoai13aa1n04x5               g158(.a(new_n253), .b(new_n250), .c(new_n236), .d(new_n237), .o1(new_n254));
  tech160nm_fixorc02aa1n04x5   g159(.a(\a[23] ), .b(\b[22] ), .out0(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n254), .c(new_n206), .d(new_n251), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n250), .o1(new_n257));
  nanb02aa1n02x5               g162(.a(new_n255), .b(new_n253), .out0(new_n258));
  aoi012aa1n02x5               g163(.a(new_n258), .b(new_n238), .c(new_n257), .o1(new_n259));
  aobi12aa1n02x5               g164(.a(new_n259), .b(new_n206), .c(new_n251), .out0(new_n260));
  norb02aa1n02x5               g165(.a(new_n256), .b(new_n260), .out0(\s[23] ));
  xorc02aa1n02x5               g166(.a(\a[24] ), .b(\b[23] ), .out0(new_n262));
  norp02aa1n02x5               g167(.a(\b[22] ), .b(\a[23] ), .o1(new_n263));
  norp02aa1n02x5               g168(.a(new_n262), .b(new_n263), .o1(new_n264));
  tech160nm_fioai012aa1n03p5x5 g169(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .o1(new_n265));
  aoi022aa1n02x7               g170(.a(new_n265), .b(new_n262), .c(new_n256), .d(new_n264), .o1(\s[24] ));
  and002aa1n02x5               g171(.a(new_n262), .b(new_n255), .o(new_n267));
  nano22aa1n03x7               g172(.a(new_n231), .b(new_n257), .c(new_n267), .out0(new_n268));
  inv000aa1d42x5               g173(.a(\a[24] ), .o1(new_n269));
  inv000aa1d42x5               g174(.a(\b[23] ), .o1(new_n270));
  oaoi03aa1n02x5               g175(.a(new_n269), .b(new_n270), .c(new_n263), .o1(new_n271));
  aob012aa1n02x5               g176(.a(new_n271), .b(new_n254), .c(new_n267), .out0(new_n272));
  xorc02aa1n02x5               g177(.a(\a[25] ), .b(\b[24] ), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n272), .c(new_n206), .d(new_n268), .o1(new_n274));
  inv000aa1n02x5               g179(.a(new_n271), .o1(new_n275));
  aoi112aa1n02x5               g180(.a(new_n273), .b(new_n275), .c(new_n254), .d(new_n267), .o1(new_n276));
  aobi12aa1n02x5               g181(.a(new_n276), .b(new_n268), .c(new_n206), .out0(new_n277));
  norb02aa1n02x5               g182(.a(new_n274), .b(new_n277), .out0(\s[25] ));
  xorc02aa1n02x5               g183(.a(\a[26] ), .b(\b[25] ), .out0(new_n279));
  norp02aa1n02x5               g184(.a(\b[24] ), .b(\a[25] ), .o1(new_n280));
  norp02aa1n02x5               g185(.a(new_n279), .b(new_n280), .o1(new_n281));
  inv000aa1d42x5               g186(.a(\a[25] ), .o1(new_n282));
  oaib12aa1n06x5               g187(.a(new_n274), .b(\b[24] ), .c(new_n282), .out0(new_n283));
  aoi022aa1n02x7               g188(.a(new_n283), .b(new_n279), .c(new_n274), .d(new_n281), .o1(\s[26] ));
  inv000aa1d42x5               g189(.a(\a[26] ), .o1(new_n285));
  xroi22aa1d04x5               g190(.a(new_n282), .b(\b[24] ), .c(new_n285), .d(\b[25] ), .out0(new_n286));
  aoai13aa1n09x5               g191(.a(new_n286), .b(new_n275), .c(new_n254), .d(new_n267), .o1(new_n287));
  norp02aa1n02x5               g192(.a(\b[26] ), .b(\a[27] ), .o1(new_n288));
  and002aa1n02x5               g193(.a(\b[26] ), .b(\a[27] ), .o(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n288), .o1(new_n290));
  nano32aa1n03x7               g195(.a(new_n231), .b(new_n286), .c(new_n257), .d(new_n267), .out0(new_n291));
  inv000aa1d42x5               g196(.a(\b[25] ), .o1(new_n292));
  oaoi03aa1n02x5               g197(.a(new_n285), .b(new_n292), .c(new_n280), .o1(new_n293));
  inv000aa1n02x5               g198(.a(new_n293), .o1(new_n294));
  aoi112aa1n02x5               g199(.a(new_n294), .b(new_n290), .c(new_n206), .d(new_n291), .o1(new_n295));
  nand22aa1n03x5               g200(.a(new_n206), .b(new_n291), .o1(new_n296));
  nand23aa1n04x5               g201(.a(new_n296), .b(new_n287), .c(new_n293), .o1(new_n297));
  aoi022aa1n02x5               g202(.a(new_n297), .b(new_n290), .c(new_n295), .d(new_n287), .o1(\s[27] ));
  inv000aa1d42x5               g203(.a(\b[26] ), .o1(new_n299));
  oaib12aa1n03x5               g204(.a(new_n297), .b(new_n299), .c(\a[27] ), .out0(new_n300));
  inv000aa1n03x5               g205(.a(new_n288), .o1(new_n301));
  tech160nm_fiaoi012aa1n05x5   g206(.a(new_n294), .b(new_n206), .c(new_n291), .o1(new_n302));
  aoai13aa1n02x7               g207(.a(new_n301), .b(new_n289), .c(new_n302), .d(new_n287), .o1(new_n303));
  xorc02aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  norp02aa1n02x5               g209(.a(new_n304), .b(new_n288), .o1(new_n305));
  aoi022aa1n03x5               g210(.a(new_n303), .b(new_n304), .c(new_n300), .d(new_n305), .o1(\s[28] ));
  inv000aa1d42x5               g211(.a(\a[28] ), .o1(new_n307));
  xroi22aa1d04x5               g212(.a(\a[27] ), .b(new_n299), .c(new_n307), .d(\b[27] ), .out0(new_n308));
  nanp02aa1n03x5               g213(.a(new_n297), .b(new_n308), .o1(new_n309));
  inv000aa1n06x5               g214(.a(new_n308), .o1(new_n310));
  oao003aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n311));
  aoai13aa1n03x5               g216(.a(new_n311), .b(new_n310), .c(new_n302), .d(new_n287), .o1(new_n312));
  xorc02aa1n02x5               g217(.a(\a[29] ), .b(\b[28] ), .out0(new_n313));
  norb02aa1n02x5               g218(.a(new_n311), .b(new_n313), .out0(new_n314));
  aoi022aa1n03x5               g219(.a(new_n312), .b(new_n313), .c(new_n309), .d(new_n314), .o1(\s[29] ));
  xorb03aa1n02x5               g220(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g221(.a(new_n304), .b(new_n313), .c(new_n290), .o(new_n317));
  nanp02aa1n03x5               g222(.a(new_n297), .b(new_n317), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n317), .o1(new_n319));
  inv000aa1d42x5               g224(.a(\b[28] ), .o1(new_n320));
  inv000aa1d42x5               g225(.a(\a[29] ), .o1(new_n321));
  oaib12aa1n02x5               g226(.a(new_n311), .b(\b[28] ), .c(new_n321), .out0(new_n322));
  oaib12aa1n02x5               g227(.a(new_n322), .b(new_n320), .c(\a[29] ), .out0(new_n323));
  aoai13aa1n03x5               g228(.a(new_n323), .b(new_n319), .c(new_n302), .d(new_n287), .o1(new_n324));
  xorc02aa1n02x5               g229(.a(\a[30] ), .b(\b[29] ), .out0(new_n325));
  oaoi13aa1n02x5               g230(.a(new_n325), .b(new_n322), .c(new_n321), .d(new_n320), .o1(new_n326));
  aoi022aa1n03x5               g231(.a(new_n324), .b(new_n325), .c(new_n318), .d(new_n326), .o1(\s[30] ));
  nano22aa1n02x4               g232(.a(new_n310), .b(new_n313), .c(new_n325), .out0(new_n328));
  nand02aa1n02x5               g233(.a(new_n297), .b(new_n328), .o1(new_n329));
  aoi022aa1n02x5               g234(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n330));
  norb02aa1n02x5               g235(.a(\b[30] ), .b(\a[31] ), .out0(new_n331));
  obai22aa1n02x7               g236(.a(\a[31] ), .b(\b[30] ), .c(\a[30] ), .d(\b[29] ), .out0(new_n332));
  aoi112aa1n02x5               g237(.a(new_n332), .b(new_n331), .c(new_n322), .d(new_n330), .o1(new_n333));
  inv000aa1n02x5               g238(.a(new_n328), .o1(new_n334));
  norp02aa1n02x5               g239(.a(\b[29] ), .b(\a[30] ), .o1(new_n335));
  aoi012aa1n02x5               g240(.a(new_n335), .b(new_n322), .c(new_n330), .o1(new_n336));
  aoai13aa1n03x5               g241(.a(new_n336), .b(new_n334), .c(new_n302), .d(new_n287), .o1(new_n337));
  xorc02aa1n02x5               g242(.a(\a[31] ), .b(\b[30] ), .out0(new_n338));
  aoi022aa1n03x5               g243(.a(new_n337), .b(new_n338), .c(new_n333), .d(new_n329), .o1(\s[31] ));
  xobna2aa1n03x5               g244(.a(new_n198), .b(new_n100), .c(new_n98), .out0(\s[3] ));
  nanp02aa1n02x5               g245(.a(new_n101), .b(new_n102), .o1(new_n341));
  xorc02aa1n02x5               g246(.a(\a[4] ), .b(\b[3] ), .out0(new_n342));
  xnbna2aa1n03x5               g247(.a(new_n342), .b(new_n341), .c(new_n196), .out0(\s[4] ));
  nanp02aa1n02x5               g248(.a(\b[3] ), .b(\a[4] ), .o1(new_n344));
  norb02aa1n02x5               g249(.a(new_n105), .b(new_n107), .out0(new_n345));
  xobna2aa1n03x5               g250(.a(new_n345), .b(new_n199), .c(new_n344), .out0(\s[5] ));
  aob012aa1n02x5               g251(.a(new_n345), .b(new_n199), .c(new_n344), .out0(new_n347));
  norb02aa1n02x5               g252(.a(new_n106), .b(new_n115), .out0(new_n348));
  xobna2aa1n03x5               g253(.a(new_n348), .b(new_n347), .c(new_n105), .out0(\s[6] ));
  inv000aa1d42x5               g254(.a(new_n115), .o1(new_n350));
  norb02aa1n02x5               g255(.a(new_n114), .b(new_n113), .out0(new_n351));
  nanp03aa1n02x5               g256(.a(new_n347), .b(new_n105), .c(new_n348), .o1(new_n352));
  xnbna2aa1n03x5               g257(.a(new_n351), .b(new_n352), .c(new_n350), .out0(\s[7] ));
  aob012aa1n02x5               g258(.a(new_n351), .b(new_n352), .c(new_n350), .out0(new_n354));
  oai012aa1n02x5               g259(.a(new_n354), .b(\b[6] ), .c(\a[7] ), .o1(new_n355));
  norp02aa1n02x5               g260(.a(new_n120), .b(new_n113), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n355), .b(new_n120), .c(new_n354), .d(new_n356), .o1(\s[8] ));
  aoi113aa1n02x5               g262(.a(new_n121), .b(new_n129), .c(new_n118), .d(new_n120), .e(new_n119), .o1(new_n358));
  aoi022aa1n02x5               g263(.a(new_n123), .b(new_n129), .c(new_n203), .d(new_n358), .o1(\s[9] ));
endmodule


