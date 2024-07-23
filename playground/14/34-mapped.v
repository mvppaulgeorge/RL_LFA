// Benchmark "adder" written by ABC on Wed Jul 17 19:26:23 2024

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
    new_n133, new_n134, new_n135, new_n136, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n167, new_n168, new_n169, new_n170, new_n171, new_n172,
    new_n173, new_n174, new_n175, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n182, new_n183, new_n184, new_n185, new_n186, new_n187,
    new_n188, new_n189, new_n191, new_n192, new_n193, new_n194, new_n196,
    new_n197, new_n198, new_n199, new_n200, new_n201, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n299, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n329,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n339, new_n340, new_n342, new_n344, new_n345, new_n346, new_n348,
    new_n349, new_n351;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor002aa1d32x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n12x5               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n06x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  nor042aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanp02aa1n04x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n03x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  nand42aa1d28x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nor002aa1n04x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb03aa1n09x5               g011(.a(new_n105), .b(new_n102), .c(new_n106), .out0(new_n107));
  aoai13aa1n12x5               g012(.a(new_n107), .b(new_n104), .c(new_n99), .d(new_n101), .o1(new_n108));
  nor042aa1n06x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  inv000aa1n06x5               g014(.a(new_n109), .o1(new_n110));
  oai122aa1n02x7               g015(.a(new_n110), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n111));
  tech160nm_fioai012aa1n04x5   g016(.a(new_n105), .b(\b[7] ), .c(\a[8] ), .o1(new_n112));
  aoi022aa1d24x5               g017(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n113));
  aoi022aa1n02x7               g018(.a(\b[7] ), .b(\a[8] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  nor043aa1n03x5               g020(.a(new_n111), .b(new_n115), .c(new_n112), .o1(new_n116));
  oaoi03aa1n02x5               g021(.a(\a[8] ), .b(\b[7] ), .c(new_n110), .o1(new_n117));
  inv040aa1n03x5               g022(.a(new_n117), .o1(new_n118));
  nor042aa1n02x5               g023(.a(\b[4] ), .b(\a[5] ), .o1(new_n119));
  nor042aa1n04x5               g024(.a(\b[5] ), .b(\a[6] ), .o1(new_n120));
  nand42aa1n16x5               g025(.a(\b[5] ), .b(\a[6] ), .o1(new_n121));
  norb03aa1n03x5               g026(.a(new_n121), .b(new_n119), .c(new_n120), .out0(new_n122));
  norp02aa1n02x5               g027(.a(\b[7] ), .b(\a[8] ), .o1(new_n123));
  nand42aa1n03x5               g028(.a(\b[7] ), .b(\a[8] ), .o1(new_n124));
  nona23aa1n06x5               g029(.a(new_n113), .b(new_n124), .c(new_n123), .d(new_n109), .out0(new_n125));
  oaih12aa1n06x5               g030(.a(new_n118), .b(new_n125), .c(new_n122), .o1(new_n126));
  xnrc02aa1n12x5               g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n126), .c(new_n116), .d(new_n108), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[9] ), .b(\a[10] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  xnbna2aa1n03x5               g036(.a(new_n131), .b(new_n129), .c(new_n98), .out0(\s[10] ));
  nor002aa1n04x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nanp02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  oai012aa1n06x5               g039(.a(new_n134), .b(new_n133), .c(new_n97), .o1(new_n135));
  aoai13aa1n04x5               g040(.a(new_n135), .b(new_n130), .c(new_n129), .d(new_n98), .o1(new_n136));
  xorb03aa1n02x5               g041(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n12x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  nand42aa1n08x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nor022aa1n08x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  nand02aa1n06x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  aoai13aa1n02x5               g047(.a(new_n142), .b(new_n138), .c(new_n136), .d(new_n139), .o1(new_n143));
  aoi112aa1n02x5               g048(.a(new_n138), .b(new_n142), .c(new_n136), .d(new_n139), .o1(new_n144));
  norb02aa1n02x5               g049(.a(new_n143), .b(new_n144), .out0(\s[12] ));
  nona23aa1n02x4               g050(.a(new_n141), .b(new_n139), .c(new_n138), .d(new_n140), .out0(new_n146));
  nor043aa1n03x5               g051(.a(new_n146), .b(new_n130), .c(new_n127), .o1(new_n147));
  aoai13aa1n06x5               g052(.a(new_n147), .b(new_n126), .c(new_n116), .d(new_n108), .o1(new_n148));
  inv000aa1n04x5               g053(.a(new_n135), .o1(new_n149));
  nano23aa1n09x5               g054(.a(new_n138), .b(new_n140), .c(new_n141), .d(new_n139), .out0(new_n150));
  ao0012aa1n03x5               g055(.a(new_n140), .b(new_n138), .c(new_n141), .o(new_n151));
  aoi012aa1n09x5               g056(.a(new_n151), .b(new_n150), .c(new_n149), .o1(new_n152));
  nor022aa1n16x5               g057(.a(\b[12] ), .b(\a[13] ), .o1(new_n153));
  nand42aa1n03x5               g058(.a(\b[12] ), .b(\a[13] ), .o1(new_n154));
  nanb02aa1n02x5               g059(.a(new_n153), .b(new_n154), .out0(new_n155));
  xobna2aa1n03x5               g060(.a(new_n155), .b(new_n148), .c(new_n152), .out0(\s[13] ));
  inv000aa1n06x5               g061(.a(new_n153), .o1(new_n157));
  aoai13aa1n02x5               g062(.a(new_n157), .b(new_n155), .c(new_n148), .d(new_n152), .o1(new_n158));
  xorb03aa1n02x5               g063(.a(new_n158), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n08x5               g064(.a(\b[13] ), .b(\a[14] ), .o1(new_n160));
  nand42aa1n03x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nona23aa1n02x4               g066(.a(new_n161), .b(new_n154), .c(new_n153), .d(new_n160), .out0(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[14] ), .b(\b[13] ), .c(new_n157), .o1(new_n163));
  inv000aa1n02x5               g068(.a(new_n163), .o1(new_n164));
  aoai13aa1n04x5               g069(.a(new_n164), .b(new_n162), .c(new_n148), .d(new_n152), .o1(new_n165));
  xorb03aa1n02x5               g070(.a(new_n165), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  inv040aa1d32x5               g071(.a(\a[15] ), .o1(new_n167));
  inv040aa1n16x5               g072(.a(\b[14] ), .o1(new_n168));
  nand22aa1n09x5               g073(.a(new_n168), .b(new_n167), .o1(new_n169));
  inv000aa1d42x5               g074(.a(new_n169), .o1(new_n170));
  nand02aa1n04x5               g075(.a(\b[14] ), .b(\a[15] ), .o1(new_n171));
  xnrc02aa1n12x5               g076(.a(\b[15] ), .b(\a[16] ), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  aoai13aa1n06x5               g078(.a(new_n173), .b(new_n170), .c(new_n165), .d(new_n171), .o1(new_n174));
  aoi112aa1n03x5               g079(.a(new_n170), .b(new_n173), .c(new_n165), .d(new_n171), .o1(new_n175));
  norb02aa1n03x4               g080(.a(new_n174), .b(new_n175), .out0(\s[16] ));
  aoi012aa1n12x5               g081(.a(new_n126), .b(new_n116), .c(new_n108), .o1(new_n177));
  nand22aa1n02x5               g082(.a(new_n169), .b(new_n171), .o1(new_n178));
  nor043aa1n02x5               g083(.a(new_n162), .b(new_n178), .c(new_n172), .o1(new_n179));
  nand02aa1d04x5               g084(.a(new_n179), .b(new_n147), .o1(new_n180));
  oabi12aa1n02x5               g085(.a(new_n151), .b(new_n146), .c(new_n135), .out0(new_n181));
  inv000aa1d42x5               g086(.a(\a[16] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[15] ), .o1(new_n183));
  nanp02aa1n02x5               g088(.a(new_n183), .b(new_n182), .o1(new_n184));
  and002aa1n02x5               g089(.a(\b[15] ), .b(\a[16] ), .o(new_n185));
  oai112aa1n03x5               g090(.a(new_n161), .b(new_n171), .c(new_n160), .d(new_n153), .o1(new_n186));
  aoai13aa1n02x5               g091(.a(new_n184), .b(new_n185), .c(new_n186), .d(new_n169), .o1(new_n187));
  tech160nm_fiaoi012aa1n05x5   g092(.a(new_n187), .b(new_n181), .c(new_n179), .o1(new_n188));
  oai012aa1n12x5               g093(.a(new_n188), .b(new_n177), .c(new_n180), .o1(new_n189));
  xorb03aa1n02x5               g094(.a(new_n189), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g095(.a(\a[18] ), .o1(new_n191));
  inv040aa1d32x5               g096(.a(\a[17] ), .o1(new_n192));
  inv000aa1d42x5               g097(.a(\b[16] ), .o1(new_n193));
  oaoi03aa1n03x5               g098(.a(new_n192), .b(new_n193), .c(new_n189), .o1(new_n194));
  xorb03aa1n02x5               g099(.a(new_n194), .b(\b[17] ), .c(new_n191), .out0(\s[18] ));
  nand02aa1n03x5               g100(.a(new_n101), .b(new_n99), .o1(new_n196));
  norb02aa1n06x5               g101(.a(new_n103), .b(new_n102), .out0(new_n197));
  aobi12aa1n06x5               g102(.a(new_n107), .b(new_n197), .c(new_n196), .out0(new_n198));
  nor043aa1n03x5               g103(.a(new_n120), .b(new_n119), .c(new_n109), .o1(new_n199));
  aob012aa1d18x5               g104(.a(new_n121), .b(\b[6] ), .c(\a[7] ), .out0(new_n200));
  nona23aa1n03x5               g105(.a(new_n199), .b(new_n114), .c(new_n200), .d(new_n112), .out0(new_n201));
  inv040aa1n02x5               g106(.a(new_n120), .o1(new_n202));
  oai112aa1n02x5               g107(.a(new_n202), .b(new_n121), .c(\b[4] ), .d(\a[5] ), .o1(new_n203));
  nor042aa1n02x5               g108(.a(new_n200), .b(new_n109), .o1(new_n204));
  norb02aa1n03x4               g109(.a(new_n124), .b(new_n123), .out0(new_n205));
  aoi013aa1n06x4               g110(.a(new_n117), .b(new_n204), .c(new_n203), .d(new_n205), .o1(new_n206));
  oai012aa1n12x5               g111(.a(new_n206), .b(new_n198), .c(new_n201), .o1(new_n207));
  nano23aa1n09x5               g112(.a(new_n153), .b(new_n160), .c(new_n161), .d(new_n154), .out0(new_n208));
  nona22aa1n09x5               g113(.a(new_n208), .b(new_n172), .c(new_n178), .out0(new_n209));
  nano32aa1n09x5               g114(.a(new_n209), .b(new_n131), .c(new_n128), .d(new_n150), .out0(new_n210));
  nanp02aa1n02x5               g115(.a(new_n186), .b(new_n169), .o1(new_n211));
  tech160nm_fioaoi03aa1n03p5x5 g116(.a(new_n182), .b(new_n183), .c(new_n211), .o1(new_n212));
  oai012aa1n09x5               g117(.a(new_n212), .b(new_n152), .c(new_n209), .o1(new_n213));
  xroi22aa1d04x5               g118(.a(new_n192), .b(\b[16] ), .c(new_n191), .d(\b[17] ), .out0(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n213), .c(new_n207), .d(new_n210), .o1(new_n215));
  oai022aa1n02x5               g120(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n216));
  oaib12aa1n02x5               g121(.a(new_n216), .b(new_n191), .c(\b[17] ), .out0(new_n217));
  nor042aa1n09x5               g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nand42aa1n06x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  norb02aa1n02x5               g124(.a(new_n219), .b(new_n218), .out0(new_n220));
  xnbna2aa1n03x5               g125(.a(new_n220), .b(new_n215), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g126(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv030aa1n04x5               g127(.a(new_n218), .o1(new_n223));
  nanp02aa1n02x5               g128(.a(new_n193), .b(new_n192), .o1(new_n224));
  oaoi03aa1n02x5               g129(.a(\a[18] ), .b(\b[17] ), .c(new_n224), .o1(new_n225));
  aoai13aa1n03x5               g130(.a(new_n220), .b(new_n225), .c(new_n189), .d(new_n214), .o1(new_n226));
  nor022aa1n06x5               g131(.a(\b[19] ), .b(\a[20] ), .o1(new_n227));
  nand42aa1n06x5               g132(.a(\b[19] ), .b(\a[20] ), .o1(new_n228));
  nanb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(new_n229));
  tech160nm_fiaoi012aa1n02p5x5 g134(.a(new_n229), .b(new_n226), .c(new_n223), .o1(new_n230));
  aobi12aa1n02x7               g135(.a(new_n220), .b(new_n215), .c(new_n217), .out0(new_n231));
  nano22aa1n02x4               g136(.a(new_n231), .b(new_n223), .c(new_n229), .out0(new_n232));
  nor002aa1n02x5               g137(.a(new_n230), .b(new_n232), .o1(\s[20] ));
  nona23aa1n02x4               g138(.a(new_n228), .b(new_n219), .c(new_n218), .d(new_n227), .out0(new_n234));
  norb02aa1n02x5               g139(.a(new_n214), .b(new_n234), .out0(new_n235));
  aoai13aa1n06x5               g140(.a(new_n235), .b(new_n213), .c(new_n207), .d(new_n210), .o1(new_n236));
  nano23aa1n09x5               g141(.a(new_n218), .b(new_n227), .c(new_n228), .d(new_n219), .out0(new_n237));
  oaoi03aa1n02x5               g142(.a(\a[20] ), .b(\b[19] ), .c(new_n223), .o1(new_n238));
  aoi012aa1n06x5               g143(.a(new_n238), .b(new_n237), .c(new_n225), .o1(new_n239));
  nor002aa1d24x5               g144(.a(\b[20] ), .b(\a[21] ), .o1(new_n240));
  nand42aa1n03x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norb02aa1n02x5               g146(.a(new_n241), .b(new_n240), .out0(new_n242));
  xnbna2aa1n03x5               g147(.a(new_n242), .b(new_n236), .c(new_n239), .out0(\s[21] ));
  inv000aa1d42x5               g148(.a(new_n240), .o1(new_n244));
  oabi12aa1n06x5               g149(.a(new_n238), .b(new_n234), .c(new_n217), .out0(new_n245));
  aoai13aa1n03x5               g150(.a(new_n242), .b(new_n245), .c(new_n189), .d(new_n235), .o1(new_n246));
  nor042aa1n04x5               g151(.a(\b[21] ), .b(\a[22] ), .o1(new_n247));
  nand42aa1n06x5               g152(.a(\b[21] ), .b(\a[22] ), .o1(new_n248));
  nanb02aa1n02x5               g153(.a(new_n247), .b(new_n248), .out0(new_n249));
  tech160nm_fiaoi012aa1n02p5x5 g154(.a(new_n249), .b(new_n246), .c(new_n244), .o1(new_n250));
  aobi12aa1n02x7               g155(.a(new_n242), .b(new_n236), .c(new_n239), .out0(new_n251));
  nano22aa1n03x5               g156(.a(new_n251), .b(new_n244), .c(new_n249), .out0(new_n252));
  norp02aa1n03x5               g157(.a(new_n250), .b(new_n252), .o1(\s[22] ));
  nano23aa1n06x5               g158(.a(new_n240), .b(new_n247), .c(new_n248), .d(new_n241), .out0(new_n254));
  and003aa1n02x5               g159(.a(new_n214), .b(new_n254), .c(new_n237), .o(new_n255));
  aoai13aa1n06x5               g160(.a(new_n255), .b(new_n213), .c(new_n207), .d(new_n210), .o1(new_n256));
  oaoi03aa1n02x5               g161(.a(\a[22] ), .b(\b[21] ), .c(new_n244), .o1(new_n257));
  aoi012aa1n06x5               g162(.a(new_n257), .b(new_n245), .c(new_n254), .o1(new_n258));
  orn002aa1n24x5               g163(.a(\a[23] ), .b(\b[22] ), .o(new_n259));
  nanp02aa1n04x5               g164(.a(\b[22] ), .b(\a[23] ), .o1(new_n260));
  nand22aa1n12x5               g165(.a(new_n259), .b(new_n260), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n256), .c(new_n258), .out0(\s[23] ));
  inv000aa1n02x5               g168(.a(new_n258), .o1(new_n264));
  aoai13aa1n03x5               g169(.a(new_n262), .b(new_n264), .c(new_n189), .d(new_n255), .o1(new_n265));
  xnrc02aa1n02x5               g170(.a(\b[23] ), .b(\a[24] ), .out0(new_n266));
  aoi012aa1n03x5               g171(.a(new_n266), .b(new_n265), .c(new_n259), .o1(new_n267));
  aoi012aa1n03x5               g172(.a(new_n261), .b(new_n256), .c(new_n258), .o1(new_n268));
  nano22aa1n03x5               g173(.a(new_n268), .b(new_n259), .c(new_n266), .out0(new_n269));
  norp02aa1n03x5               g174(.a(new_n267), .b(new_n269), .o1(\s[24] ));
  nona22aa1n09x5               g175(.a(new_n254), .b(new_n266), .c(new_n261), .out0(new_n271));
  nano22aa1n02x4               g176(.a(new_n271), .b(new_n214), .c(new_n237), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n213), .c(new_n207), .d(new_n210), .o1(new_n273));
  inv000aa1d42x5               g178(.a(\a[24] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(\b[23] ), .o1(new_n275));
  aoai13aa1n06x5               g180(.a(new_n260), .b(new_n247), .c(new_n240), .d(new_n248), .o1(new_n276));
  nand42aa1n02x5               g181(.a(new_n276), .b(new_n259), .o1(new_n277));
  oaoi03aa1n12x5               g182(.a(new_n274), .b(new_n275), .c(new_n277), .o1(new_n278));
  oai012aa1n18x5               g183(.a(new_n278), .b(new_n239), .c(new_n271), .o1(new_n279));
  inv000aa1d42x5               g184(.a(new_n279), .o1(new_n280));
  xnrc02aa1n12x5               g185(.a(\b[24] ), .b(\a[25] ), .out0(new_n281));
  inv000aa1d42x5               g186(.a(new_n281), .o1(new_n282));
  xnbna2aa1n03x5               g187(.a(new_n282), .b(new_n273), .c(new_n280), .out0(\s[25] ));
  nor042aa1n03x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n284), .o1(new_n285));
  aoai13aa1n03x5               g190(.a(new_n282), .b(new_n279), .c(new_n189), .d(new_n272), .o1(new_n286));
  tech160nm_fixnrc02aa1n04x5   g191(.a(\b[25] ), .b(\a[26] ), .out0(new_n287));
  aoi012aa1n03x5               g192(.a(new_n287), .b(new_n286), .c(new_n285), .o1(new_n288));
  tech160nm_fiaoi012aa1n03p5x5 g193(.a(new_n281), .b(new_n273), .c(new_n280), .o1(new_n289));
  nano22aa1n02x5               g194(.a(new_n289), .b(new_n285), .c(new_n287), .out0(new_n290));
  norp02aa1n03x5               g195(.a(new_n288), .b(new_n290), .o1(\s[26] ));
  nor042aa1d18x5               g196(.a(new_n287), .b(new_n281), .o1(new_n292));
  nano32aa1n03x7               g197(.a(new_n271), .b(new_n214), .c(new_n292), .d(new_n237), .out0(new_n293));
  aoai13aa1n06x5               g198(.a(new_n293), .b(new_n213), .c(new_n207), .d(new_n210), .o1(new_n294));
  oao003aa1n02x5               g199(.a(\a[26] ), .b(\b[25] ), .c(new_n285), .carry(new_n295));
  aobi12aa1n09x5               g200(.a(new_n295), .b(new_n279), .c(new_n292), .out0(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  xnbna2aa1n06x5               g202(.a(new_n297), .b(new_n294), .c(new_n296), .out0(\s[27] ));
  norp02aa1n02x5               g203(.a(\b[26] ), .b(\a[27] ), .o1(new_n299));
  inv040aa1n03x5               g204(.a(new_n299), .o1(new_n300));
  nona23aa1n02x4               g205(.a(new_n248), .b(new_n241), .c(new_n240), .d(new_n247), .out0(new_n301));
  norp03aa1n02x5               g206(.a(new_n301), .b(new_n261), .c(new_n266), .o1(new_n302));
  nanp02aa1n04x5               g207(.a(new_n245), .b(new_n302), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n292), .o1(new_n304));
  aoai13aa1n06x5               g209(.a(new_n295), .b(new_n304), .c(new_n303), .d(new_n278), .o1(new_n305));
  aoai13aa1n03x5               g210(.a(new_n297), .b(new_n305), .c(new_n189), .d(new_n293), .o1(new_n306));
  xnrc02aa1n02x5               g211(.a(\b[27] ), .b(\a[28] ), .out0(new_n307));
  aoi012aa1n03x5               g212(.a(new_n307), .b(new_n306), .c(new_n300), .o1(new_n308));
  aobi12aa1n06x5               g213(.a(new_n297), .b(new_n294), .c(new_n296), .out0(new_n309));
  nano22aa1n03x5               g214(.a(new_n309), .b(new_n300), .c(new_n307), .out0(new_n310));
  norp02aa1n03x5               g215(.a(new_n308), .b(new_n310), .o1(\s[28] ));
  norb02aa1n02x5               g216(.a(new_n297), .b(new_n307), .out0(new_n312));
  aoai13aa1n02x7               g217(.a(new_n312), .b(new_n305), .c(new_n189), .d(new_n293), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[28] ), .b(\b[27] ), .c(new_n300), .carry(new_n314));
  xnrc02aa1n02x5               g219(.a(\b[28] ), .b(\a[29] ), .out0(new_n315));
  aoi012aa1n03x5               g220(.a(new_n315), .b(new_n313), .c(new_n314), .o1(new_n316));
  aobi12aa1n06x5               g221(.a(new_n312), .b(new_n294), .c(new_n296), .out0(new_n317));
  nano22aa1n03x5               g222(.a(new_n317), .b(new_n314), .c(new_n315), .out0(new_n318));
  norp02aa1n03x5               g223(.a(new_n316), .b(new_n318), .o1(\s[29] ));
  xorb03aa1n02x5               g224(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g225(.a(new_n297), .b(new_n315), .c(new_n307), .out0(new_n321));
  aoai13aa1n02x5               g226(.a(new_n321), .b(new_n305), .c(new_n189), .d(new_n293), .o1(new_n322));
  oao003aa1n02x5               g227(.a(\a[29] ), .b(\b[28] ), .c(new_n314), .carry(new_n323));
  xnrc02aa1n02x5               g228(.a(\b[29] ), .b(\a[30] ), .out0(new_n324));
  aoi012aa1n03x5               g229(.a(new_n324), .b(new_n322), .c(new_n323), .o1(new_n325));
  aobi12aa1n06x5               g230(.a(new_n321), .b(new_n294), .c(new_n296), .out0(new_n326));
  nano22aa1n03x5               g231(.a(new_n326), .b(new_n323), .c(new_n324), .out0(new_n327));
  norp02aa1n03x5               g232(.a(new_n325), .b(new_n327), .o1(\s[30] ));
  xnrc02aa1n02x5               g233(.a(\b[30] ), .b(\a[31] ), .out0(new_n329));
  nona32aa1n02x4               g234(.a(new_n297), .b(new_n324), .c(new_n315), .d(new_n307), .out0(new_n330));
  inv000aa1n02x5               g235(.a(new_n330), .o1(new_n331));
  aoai13aa1n02x5               g236(.a(new_n331), .b(new_n305), .c(new_n189), .d(new_n293), .o1(new_n332));
  oao003aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .c(new_n323), .carry(new_n333));
  aoi012aa1n03x5               g238(.a(new_n329), .b(new_n332), .c(new_n333), .o1(new_n334));
  tech160nm_fiaoi012aa1n02p5x5 g239(.a(new_n330), .b(new_n294), .c(new_n296), .o1(new_n335));
  nano22aa1n03x5               g240(.a(new_n335), .b(new_n329), .c(new_n333), .out0(new_n336));
  norp02aa1n03x5               g241(.a(new_n334), .b(new_n336), .o1(\s[31] ));
  xnbna2aa1n03x5               g242(.a(new_n197), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  norb02aa1n02x5               g243(.a(new_n105), .b(new_n106), .out0(new_n339));
  aoi012aa1n02x5               g244(.a(new_n102), .b(new_n196), .c(new_n197), .o1(new_n340));
  oai012aa1n02x5               g245(.a(new_n108), .b(new_n340), .c(new_n339), .o1(\s[4] ));
  xorc02aa1n02x5               g246(.a(\a[5] ), .b(\b[4] ), .out0(new_n342));
  xobna2aa1n03x5               g247(.a(new_n342), .b(new_n108), .c(new_n105), .out0(\s[5] ));
  aoi013aa1n02x4               g248(.a(new_n119), .b(new_n108), .c(new_n105), .d(new_n342), .o1(new_n344));
  nanp03aa1n02x5               g249(.a(new_n108), .b(new_n105), .c(new_n342), .o1(new_n345));
  nanp02aa1n02x5               g250(.a(new_n345), .b(new_n122), .o1(new_n346));
  aoai13aa1n02x5               g251(.a(new_n346), .b(new_n344), .c(new_n202), .d(new_n121), .o1(\s[6] ));
  xnrc02aa1n02x5               g252(.a(\b[6] ), .b(\a[7] ), .out0(new_n348));
  nanp02aa1n02x5               g253(.a(new_n346), .b(new_n121), .o1(new_n349));
  aoi022aa1n02x5               g254(.a(new_n349), .b(new_n348), .c(new_n204), .d(new_n346), .o1(\s[7] ));
  aoai13aa1n02x5               g255(.a(new_n110), .b(new_n200), .c(new_n345), .d(new_n122), .o1(new_n351));
  xorb03aa1n02x5               g256(.a(new_n351), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrc02aa1n02x5               g257(.a(new_n177), .b(new_n128), .out0(\s[9] ));
endmodule


