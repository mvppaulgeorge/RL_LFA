// Benchmark "adder" written by ABC on Wed Jul 17 16:13:10 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n171, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n188, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n263, new_n264,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n271, new_n272,
    new_n273, new_n274, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n294, new_n295,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n346, new_n348, new_n349, new_n351, new_n353, new_n354,
    new_n355, new_n356, new_n358, new_n359, new_n360, new_n362, new_n363,
    new_n365;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\b[8] ), .o1(new_n97));
  nanb02aa1n02x5               g002(.a(\a[9] ), .b(new_n97), .out0(new_n98));
  norp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv000aa1n02x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n02x5               g005(.a(\b[1] ), .b(\a[2] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aob012aa1n02x5               g007(.a(new_n100), .b(new_n101), .c(new_n102), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  norp02aa1n04x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n06x4               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nand03aa1n02x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  tech160nm_fioai012aa1n05x5   g015(.a(new_n105), .b(new_n107), .c(new_n104), .o1(new_n111));
  nor042aa1n02x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nand42aa1n02x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nor002aa1n03x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[4] ), .b(\a[5] ), .o1(new_n115));
  nano23aa1n02x4               g020(.a(new_n112), .b(new_n114), .c(new_n115), .d(new_n113), .out0(new_n116));
  nor002aa1n03x5               g021(.a(\b[5] ), .b(\a[6] ), .o1(new_n117));
  nand02aa1n06x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[7] ), .b(\a[8] ), .o1(new_n119));
  nanp02aa1n03x5               g024(.a(\b[7] ), .b(\a[8] ), .o1(new_n120));
  nano23aa1n02x4               g025(.a(new_n117), .b(new_n119), .c(new_n120), .d(new_n118), .out0(new_n121));
  nand22aa1n03x5               g026(.a(new_n121), .b(new_n116), .o1(new_n122));
  nano22aa1n02x4               g027(.a(new_n112), .b(new_n113), .c(new_n120), .out0(new_n123));
  tech160nm_fioai012aa1n03p5x5 g028(.a(new_n118), .b(\b[7] ), .c(\a[8] ), .o1(new_n124));
  oab012aa1n06x5               g029(.a(new_n124), .b(new_n114), .c(new_n117), .out0(new_n125));
  tech160nm_fiaoi012aa1n03p5x5 g030(.a(new_n119), .b(new_n112), .c(new_n120), .o1(new_n126));
  inv020aa1n02x5               g031(.a(new_n126), .o1(new_n127));
  aoi012aa1n06x5               g032(.a(new_n127), .b(new_n125), .c(new_n123), .o1(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n122), .c(new_n110), .d(new_n111), .o1(new_n129));
  xorc02aa1n02x5               g034(.a(\a[9] ), .b(\b[8] ), .out0(new_n130));
  nanp02aa1n02x5               g035(.a(new_n129), .b(new_n130), .o1(new_n131));
  xorc02aa1n02x5               g036(.a(\a[10] ), .b(\b[9] ), .out0(new_n132));
  xnbna2aa1n03x5               g037(.a(new_n132), .b(new_n131), .c(new_n98), .out0(\s[10] ));
  inv000aa1d42x5               g038(.a(\a[10] ), .o1(new_n134));
  xroi22aa1d04x5               g039(.a(new_n134), .b(\b[9] ), .c(new_n97), .d(\a[9] ), .out0(new_n135));
  oaoi03aa1n09x5               g040(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n136));
  nor042aa1n06x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nand42aa1n04x5               g042(.a(\b[10] ), .b(\a[11] ), .o1(new_n138));
  norb02aa1n02x7               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  aoai13aa1n03x5               g044(.a(new_n139), .b(new_n136), .c(new_n129), .d(new_n135), .o1(new_n140));
  aoi112aa1n02x5               g045(.a(new_n139), .b(new_n136), .c(new_n129), .d(new_n135), .o1(new_n141));
  norb02aa1n02x5               g046(.a(new_n140), .b(new_n141), .out0(\s[11] ));
  orn002aa1n02x5               g047(.a(\a[11] ), .b(\b[10] ), .o(new_n143));
  nor022aa1n08x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand42aa1n08x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n06x4               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  xnbna2aa1n03x5               g051(.a(new_n146), .b(new_n140), .c(new_n143), .out0(\s[12] ));
  nano23aa1d15x5               g052(.a(new_n137), .b(new_n144), .c(new_n145), .d(new_n138), .out0(new_n148));
  inv040aa1n03x5               g053(.a(new_n148), .o1(new_n149));
  nano22aa1n03x7               g054(.a(new_n149), .b(new_n130), .c(new_n132), .out0(new_n150));
  nanp02aa1n02x5               g055(.a(new_n129), .b(new_n150), .o1(new_n151));
  aoi012aa1n02x5               g056(.a(new_n99), .b(new_n101), .c(new_n102), .o1(new_n152));
  nona23aa1n02x4               g057(.a(new_n108), .b(new_n105), .c(new_n104), .d(new_n107), .out0(new_n153));
  oai012aa1n03x5               g058(.a(new_n111), .b(new_n153), .c(new_n152), .o1(new_n154));
  nanb02aa1n06x5               g059(.a(new_n122), .b(new_n154), .out0(new_n155));
  inv000aa1n02x5               g060(.a(new_n150), .o1(new_n156));
  aoi012aa1d18x5               g061(.a(new_n144), .b(new_n137), .c(new_n145), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoi012aa1d18x5               g063(.a(new_n158), .b(new_n148), .c(new_n136), .o1(new_n159));
  aoai13aa1n02x5               g064(.a(new_n159), .b(new_n156), .c(new_n155), .d(new_n128), .o1(new_n160));
  nor042aa1n02x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  norb02aa1n02x5               g067(.a(new_n162), .b(new_n161), .out0(new_n163));
  aoi112aa1n02x5               g068(.a(new_n158), .b(new_n163), .c(new_n148), .d(new_n136), .o1(new_n164));
  aoi022aa1n02x5               g069(.a(new_n160), .b(new_n163), .c(new_n151), .d(new_n164), .o1(\s[13] ));
  orn002aa1n02x5               g070(.a(\a[13] ), .b(\b[12] ), .o(new_n166));
  inv000aa1n02x5               g071(.a(new_n159), .o1(new_n167));
  aoai13aa1n02x5               g072(.a(new_n163), .b(new_n167), .c(new_n129), .d(new_n150), .o1(new_n168));
  nor042aa1n02x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  nand42aa1n06x5               g074(.a(\b[13] ), .b(\a[14] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  xnbna2aa1n03x5               g076(.a(new_n171), .b(new_n168), .c(new_n166), .out0(\s[14] ));
  nano23aa1n03x5               g077(.a(new_n161), .b(new_n169), .c(new_n170), .d(new_n162), .out0(new_n173));
  aoai13aa1n02x5               g078(.a(new_n173), .b(new_n167), .c(new_n129), .d(new_n150), .o1(new_n174));
  aoi012aa1n06x5               g079(.a(new_n169), .b(new_n161), .c(new_n170), .o1(new_n175));
  nor042aa1n09x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  nand42aa1n03x5               g081(.a(\b[14] ), .b(\a[15] ), .o1(new_n177));
  norb02aa1n02x5               g082(.a(new_n177), .b(new_n176), .out0(new_n178));
  xnbna2aa1n03x5               g083(.a(new_n178), .b(new_n174), .c(new_n175), .out0(\s[15] ));
  inv040aa1n02x5               g084(.a(new_n175), .o1(new_n180));
  aoai13aa1n02x5               g085(.a(new_n178), .b(new_n180), .c(new_n160), .d(new_n173), .o1(new_n181));
  nor042aa1n02x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  nand22aa1n02x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  norb02aa1n02x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoib12aa1n02x5               g089(.a(new_n176), .b(new_n183), .c(new_n182), .out0(new_n185));
  inv000aa1d42x5               g090(.a(new_n176), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n178), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n186), .b(new_n187), .c(new_n174), .d(new_n175), .o1(new_n188));
  aoi022aa1n02x7               g093(.a(new_n188), .b(new_n184), .c(new_n181), .d(new_n185), .o1(\s[16] ));
  nano23aa1n03x7               g094(.a(new_n176), .b(new_n182), .c(new_n183), .d(new_n177), .out0(new_n190));
  nand22aa1n03x5               g095(.a(new_n190), .b(new_n173), .o1(new_n191));
  nano32aa1n03x7               g096(.a(new_n191), .b(new_n148), .c(new_n132), .d(new_n130), .out0(new_n192));
  nanp02aa1n02x5               g097(.a(new_n129), .b(new_n192), .o1(new_n193));
  inv020aa1n04x5               g098(.a(new_n192), .o1(new_n194));
  norp02aa1n02x5               g099(.a(\b[9] ), .b(\a[10] ), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n196));
  oai112aa1n02x5               g101(.a(new_n139), .b(new_n146), .c(new_n196), .d(new_n195), .o1(new_n197));
  aoi012aa1n02x7               g102(.a(new_n191), .b(new_n197), .c(new_n157), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n182), .b(new_n176), .c(new_n183), .o1(new_n199));
  aobi12aa1n02x7               g104(.a(new_n199), .b(new_n190), .c(new_n180), .out0(new_n200));
  norb02aa1n03x5               g105(.a(new_n200), .b(new_n198), .out0(new_n201));
  aoai13aa1n12x5               g106(.a(new_n201), .b(new_n194), .c(new_n155), .d(new_n128), .o1(new_n202));
  tech160nm_fixorc02aa1n03p5x5 g107(.a(\a[17] ), .b(\b[16] ), .out0(new_n203));
  nanb02aa1n02x5               g108(.a(new_n203), .b(new_n199), .out0(new_n204));
  aoi112aa1n02x5               g109(.a(new_n198), .b(new_n204), .c(new_n180), .d(new_n190), .o1(new_n205));
  aoi022aa1n02x5               g110(.a(new_n202), .b(new_n203), .c(new_n193), .d(new_n205), .o1(\s[17] ));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  nanb02aa1n02x5               g112(.a(\b[16] ), .b(new_n207), .out0(new_n208));
  aoai13aa1n04x5               g113(.a(new_n200), .b(new_n191), .c(new_n197), .d(new_n157), .o1(new_n209));
  aoai13aa1n02x5               g114(.a(new_n203), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n210));
  tech160nm_fixorc02aa1n03p5x5 g115(.a(\a[18] ), .b(\b[17] ), .out0(new_n211));
  xnbna2aa1n03x5               g116(.a(new_n211), .b(new_n210), .c(new_n208), .out0(\s[18] ));
  and002aa1n02x5               g117(.a(new_n211), .b(new_n203), .o(new_n213));
  aoai13aa1n04x5               g118(.a(new_n213), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n214));
  nor022aa1n02x5               g119(.a(\b[17] ), .b(\a[18] ), .o1(new_n215));
  aoi112aa1n09x5               g120(.a(\b[16] ), .b(\a[17] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n216));
  norp02aa1n02x5               g121(.a(new_n216), .b(new_n215), .o1(new_n217));
  xorc02aa1n12x5               g122(.a(\a[19] ), .b(\b[18] ), .out0(new_n218));
  xnbna2aa1n03x5               g123(.a(new_n218), .b(new_n214), .c(new_n217), .out0(\s[19] ));
  xnrc02aa1n02x5               g124(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g125(.a(new_n217), .o1(new_n221));
  aoai13aa1n02x5               g126(.a(new_n218), .b(new_n221), .c(new_n202), .d(new_n213), .o1(new_n222));
  tech160nm_fixorc02aa1n04x5   g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  nor042aa1n06x5               g128(.a(\b[18] ), .b(\a[19] ), .o1(new_n224));
  norp02aa1n02x5               g129(.a(new_n223), .b(new_n224), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n224), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n218), .o1(new_n227));
  aoai13aa1n06x5               g132(.a(new_n226), .b(new_n227), .c(new_n214), .d(new_n217), .o1(new_n228));
  aoi022aa1n02x5               g133(.a(new_n228), .b(new_n223), .c(new_n222), .d(new_n225), .o1(\s[20] ));
  nano32aa1n02x4               g134(.a(new_n227), .b(new_n223), .c(new_n203), .d(new_n211), .out0(new_n230));
  aoai13aa1n03x5               g135(.a(new_n230), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n231));
  oai112aa1n06x5               g136(.a(new_n218), .b(new_n223), .c(new_n216), .d(new_n215), .o1(new_n232));
  oao003aa1n09x5               g137(.a(\a[20] ), .b(\b[19] ), .c(new_n226), .carry(new_n233));
  nand02aa1d06x5               g138(.a(new_n232), .b(new_n233), .o1(new_n234));
  inv000aa1d42x5               g139(.a(new_n234), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n231), .b(new_n235), .o1(new_n236));
  nor002aa1d32x5               g141(.a(\b[20] ), .b(\a[21] ), .o1(new_n237));
  nanp02aa1n24x5               g142(.a(\b[20] ), .b(\a[21] ), .o1(new_n238));
  norb02aa1n02x5               g143(.a(new_n238), .b(new_n237), .out0(new_n239));
  inv000aa1d42x5               g144(.a(new_n239), .o1(new_n240));
  and003aa1n02x5               g145(.a(new_n232), .b(new_n240), .c(new_n233), .o(new_n241));
  aoi022aa1n02x5               g146(.a(new_n236), .b(new_n239), .c(new_n231), .d(new_n241), .o1(\s[21] ));
  aoai13aa1n03x5               g147(.a(new_n239), .b(new_n234), .c(new_n202), .d(new_n230), .o1(new_n243));
  nor042aa1n04x5               g148(.a(\b[21] ), .b(\a[22] ), .o1(new_n244));
  nanp02aa1n12x5               g149(.a(\b[21] ), .b(\a[22] ), .o1(new_n245));
  norb02aa1n02x5               g150(.a(new_n245), .b(new_n244), .out0(new_n246));
  aoib12aa1n02x5               g151(.a(new_n237), .b(new_n245), .c(new_n244), .out0(new_n247));
  inv000aa1n03x5               g152(.a(new_n237), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n248), .b(new_n240), .c(new_n231), .d(new_n235), .o1(new_n249));
  aoi022aa1n03x5               g154(.a(new_n249), .b(new_n246), .c(new_n243), .d(new_n247), .o1(\s[22] ));
  inv000aa1d42x5               g155(.a(\a[19] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(\a[20] ), .o1(new_n252));
  xroi22aa1d04x5               g157(.a(new_n251), .b(\b[18] ), .c(new_n252), .d(\b[19] ), .out0(new_n253));
  nano23aa1d12x5               g158(.a(new_n237), .b(new_n244), .c(new_n245), .d(new_n238), .out0(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  nano32aa1n02x4               g160(.a(new_n255), .b(new_n253), .c(new_n211), .d(new_n203), .out0(new_n256));
  aoai13aa1n03x5               g161(.a(new_n256), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n257));
  tech160nm_fioaoi03aa1n04x5   g162(.a(\a[22] ), .b(\b[21] ), .c(new_n248), .o1(new_n258));
  aoi012aa1n02x7               g163(.a(new_n258), .b(new_n234), .c(new_n254), .o1(new_n259));
  nanp02aa1n02x5               g164(.a(new_n257), .b(new_n259), .o1(new_n260));
  nor042aa1n09x5               g165(.a(\b[22] ), .b(\a[23] ), .o1(new_n261));
  nand42aa1n03x5               g166(.a(\b[22] ), .b(\a[23] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  aoi112aa1n02x5               g168(.a(new_n263), .b(new_n258), .c(new_n234), .d(new_n254), .o1(new_n264));
  aoi022aa1n02x5               g169(.a(new_n260), .b(new_n263), .c(new_n257), .d(new_n264), .o1(\s[23] ));
  inv000aa1n02x5               g170(.a(new_n259), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n263), .b(new_n266), .c(new_n202), .d(new_n256), .o1(new_n267));
  nor002aa1n03x5               g172(.a(\b[23] ), .b(\a[24] ), .o1(new_n268));
  tech160nm_finand02aa1n03p5x5 g173(.a(\b[23] ), .b(\a[24] ), .o1(new_n269));
  norb02aa1n02x5               g174(.a(new_n269), .b(new_n268), .out0(new_n270));
  aoib12aa1n02x5               g175(.a(new_n261), .b(new_n269), .c(new_n268), .out0(new_n271));
  inv000aa1d42x5               g176(.a(new_n261), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n263), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n272), .b(new_n273), .c(new_n257), .d(new_n259), .o1(new_n274));
  aoi022aa1n02x7               g179(.a(new_n274), .b(new_n270), .c(new_n267), .d(new_n271), .o1(\s[24] ));
  nano23aa1n09x5               g180(.a(new_n261), .b(new_n268), .c(new_n269), .d(new_n262), .out0(new_n276));
  nand22aa1n03x5               g181(.a(new_n276), .b(new_n254), .o1(new_n277));
  nano32aa1n02x4               g182(.a(new_n277), .b(new_n253), .c(new_n211), .d(new_n203), .out0(new_n278));
  aoai13aa1n02x7               g183(.a(new_n278), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n279));
  oaoi03aa1n02x5               g184(.a(\a[24] ), .b(\b[23] ), .c(new_n272), .o1(new_n280));
  tech160nm_fiaoi012aa1n03p5x5 g185(.a(new_n280), .b(new_n276), .c(new_n258), .o1(new_n281));
  aoai13aa1n12x5               g186(.a(new_n281), .b(new_n277), .c(new_n232), .d(new_n233), .o1(new_n282));
  inv000aa1d42x5               g187(.a(new_n282), .o1(new_n283));
  nanp02aa1n02x5               g188(.a(new_n279), .b(new_n283), .o1(new_n284));
  xorc02aa1n12x5               g189(.a(\a[25] ), .b(\b[24] ), .out0(new_n285));
  aoi112aa1n02x5               g190(.a(new_n280), .b(new_n285), .c(new_n276), .d(new_n258), .o1(new_n286));
  oa0012aa1n02x5               g191(.a(new_n286), .b(new_n235), .c(new_n277), .o(new_n287));
  aoi022aa1n02x5               g192(.a(new_n284), .b(new_n285), .c(new_n287), .d(new_n279), .o1(\s[25] ));
  aoai13aa1n03x5               g193(.a(new_n285), .b(new_n282), .c(new_n202), .d(new_n278), .o1(new_n289));
  xorc02aa1n02x5               g194(.a(\a[26] ), .b(\b[25] ), .out0(new_n290));
  nor042aa1n06x5               g195(.a(\b[24] ), .b(\a[25] ), .o1(new_n291));
  norp02aa1n02x5               g196(.a(new_n290), .b(new_n291), .o1(new_n292));
  inv000aa1d42x5               g197(.a(new_n291), .o1(new_n293));
  inv000aa1d42x5               g198(.a(new_n285), .o1(new_n294));
  aoai13aa1n03x5               g199(.a(new_n293), .b(new_n294), .c(new_n279), .d(new_n283), .o1(new_n295));
  aoi022aa1n03x5               g200(.a(new_n295), .b(new_n290), .c(new_n289), .d(new_n292), .o1(\s[26] ));
  and002aa1n18x5               g201(.a(new_n290), .b(new_n285), .o(new_n297));
  nano32aa1n03x7               g202(.a(new_n277), .b(new_n297), .c(new_n213), .d(new_n253), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n209), .c(new_n129), .d(new_n192), .o1(new_n299));
  oao003aa1n02x5               g204(.a(\a[26] ), .b(\b[25] ), .c(new_n293), .carry(new_n300));
  inv000aa1n02x5               g205(.a(new_n300), .o1(new_n301));
  tech160nm_fiaoi012aa1n05x5   g206(.a(new_n301), .b(new_n282), .c(new_n297), .o1(new_n302));
  nanp02aa1n02x5               g207(.a(new_n299), .b(new_n302), .o1(new_n303));
  xorc02aa1n12x5               g208(.a(\a[27] ), .b(\b[26] ), .out0(new_n304));
  aoi112aa1n02x5               g209(.a(new_n304), .b(new_n301), .c(new_n282), .d(new_n297), .o1(new_n305));
  aoi022aa1n02x5               g210(.a(new_n303), .b(new_n304), .c(new_n299), .d(new_n305), .o1(\s[27] ));
  aob012aa1n03x5               g211(.a(new_n300), .b(new_n282), .c(new_n297), .out0(new_n307));
  aoai13aa1n03x5               g212(.a(new_n304), .b(new_n307), .c(new_n202), .d(new_n298), .o1(new_n308));
  tech160nm_fixorc02aa1n02p5x5 g213(.a(\a[28] ), .b(\b[27] ), .out0(new_n309));
  norp02aa1n02x5               g214(.a(\b[26] ), .b(\a[27] ), .o1(new_n310));
  norp02aa1n02x5               g215(.a(new_n309), .b(new_n310), .o1(new_n311));
  inv000aa1n06x5               g216(.a(new_n310), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n304), .o1(new_n313));
  aoai13aa1n02x7               g218(.a(new_n312), .b(new_n313), .c(new_n299), .d(new_n302), .o1(new_n314));
  aoi022aa1n03x5               g219(.a(new_n314), .b(new_n309), .c(new_n308), .d(new_n311), .o1(\s[28] ));
  and002aa1n02x5               g220(.a(new_n309), .b(new_n304), .o(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n307), .c(new_n202), .d(new_n298), .o1(new_n317));
  inv000aa1d42x5               g222(.a(new_n316), .o1(new_n318));
  oaoi03aa1n12x5               g223(.a(\a[28] ), .b(\b[27] ), .c(new_n312), .o1(new_n319));
  inv000aa1d42x5               g224(.a(new_n319), .o1(new_n320));
  aoai13aa1n02x7               g225(.a(new_n320), .b(new_n318), .c(new_n299), .d(new_n302), .o1(new_n321));
  tech160nm_fixorc02aa1n02p5x5 g226(.a(\a[29] ), .b(\b[28] ), .out0(new_n322));
  norp02aa1n02x5               g227(.a(new_n319), .b(new_n322), .o1(new_n323));
  aoi022aa1n03x5               g228(.a(new_n321), .b(new_n322), .c(new_n317), .d(new_n323), .o1(\s[29] ));
  xorb03aa1n02x5               g229(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g230(.a(new_n313), .b(new_n309), .c(new_n322), .out0(new_n326));
  aoai13aa1n03x5               g231(.a(new_n326), .b(new_n307), .c(new_n202), .d(new_n298), .o1(new_n327));
  inv000aa1n02x5               g232(.a(new_n326), .o1(new_n328));
  inv000aa1d42x5               g233(.a(\b[28] ), .o1(new_n329));
  oaib12aa1n02x5               g234(.a(new_n319), .b(new_n329), .c(\a[29] ), .out0(new_n330));
  oa0012aa1n02x5               g235(.a(new_n330), .b(\b[28] ), .c(\a[29] ), .o(new_n331));
  aoai13aa1n02x7               g236(.a(new_n331), .b(new_n328), .c(new_n299), .d(new_n302), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  oabi12aa1n02x5               g238(.a(new_n333), .b(\a[29] ), .c(\b[28] ), .out0(new_n334));
  norb02aa1n02x5               g239(.a(new_n330), .b(new_n334), .out0(new_n335));
  aoi022aa1n03x5               g240(.a(new_n332), .b(new_n333), .c(new_n327), .d(new_n335), .o1(\s[30] ));
  nano32aa1n02x4               g241(.a(new_n313), .b(new_n333), .c(new_n309), .d(new_n322), .out0(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n307), .c(new_n202), .d(new_n298), .o1(new_n338));
  xorc02aa1n02x5               g243(.a(\a[31] ), .b(\b[30] ), .out0(new_n339));
  oai122aa1n02x7               g244(.a(new_n330), .b(\a[30] ), .c(\b[29] ), .d(\a[29] ), .e(\b[28] ), .o1(new_n340));
  aob012aa1n02x5               g245(.a(new_n340), .b(\b[29] ), .c(\a[30] ), .out0(new_n341));
  norb02aa1n02x5               g246(.a(new_n341), .b(new_n339), .out0(new_n342));
  inv000aa1n02x5               g247(.a(new_n337), .o1(new_n343));
  aoai13aa1n02x7               g248(.a(new_n341), .b(new_n343), .c(new_n299), .d(new_n302), .o1(new_n344));
  aoi022aa1n03x5               g249(.a(new_n344), .b(new_n339), .c(new_n338), .d(new_n342), .o1(\s[31] ));
  aoi112aa1n02x5               g250(.a(new_n109), .b(new_n99), .c(new_n101), .d(new_n102), .o1(new_n346));
  aoi012aa1n02x5               g251(.a(new_n346), .b(new_n103), .c(new_n109), .o1(\s[3] ));
  oaoi03aa1n02x5               g252(.a(\a[3] ), .b(\b[2] ), .c(new_n152), .o1(new_n348));
  aoi112aa1n02x5               g253(.a(new_n107), .b(new_n106), .c(new_n103), .d(new_n108), .o1(new_n349));
  aoi012aa1n02x5               g254(.a(new_n349), .b(new_n106), .c(new_n348), .o1(\s[4] ));
  norb02aa1n02x5               g255(.a(new_n115), .b(new_n114), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n110), .c(new_n111), .out0(\s[5] ));
  nanb02aa1n02x5               g257(.a(new_n117), .b(new_n118), .out0(new_n353));
  aoai13aa1n02x5               g258(.a(new_n353), .b(new_n114), .c(new_n154), .d(new_n115), .o1(new_n354));
  norb03aa1n02x5               g259(.a(new_n118), .b(new_n114), .c(new_n117), .out0(new_n355));
  aob012aa1n02x5               g260(.a(new_n355), .b(new_n154), .c(new_n351), .out0(new_n356));
  nanp02aa1n02x5               g261(.a(new_n354), .b(new_n356), .o1(\s[6] ));
  nano22aa1n02x4               g262(.a(new_n112), .b(new_n113), .c(new_n118), .out0(new_n358));
  nanp02aa1n02x5               g263(.a(new_n356), .b(new_n358), .o1(new_n359));
  aboi22aa1n03x5               g264(.a(new_n112), .b(new_n113), .c(new_n356), .d(new_n118), .out0(new_n360));
  norb02aa1n02x5               g265(.a(new_n359), .b(new_n360), .out0(\s[7] ));
  orn002aa1n02x5               g266(.a(\a[7] ), .b(\b[6] ), .o(new_n362));
  norb02aa1n02x5               g267(.a(new_n120), .b(new_n119), .out0(new_n363));
  xnbna2aa1n03x5               g268(.a(new_n363), .b(new_n359), .c(new_n362), .out0(\s[8] ));
  aoi112aa1n02x5               g269(.a(new_n127), .b(new_n130), .c(new_n125), .d(new_n123), .o1(new_n365));
  aoi022aa1n02x5               g270(.a(new_n129), .b(new_n130), .c(new_n155), .d(new_n365), .o1(\s[9] ));
endmodule


