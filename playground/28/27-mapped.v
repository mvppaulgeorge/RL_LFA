// Benchmark "adder" written by ABC on Thu Jul 18 02:32:38 2024

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
    new_n132, new_n133, new_n134, new_n136, new_n137, new_n138, new_n139,
    new_n140, new_n141, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n173, new_n174, new_n175, new_n176, new_n178,
    new_n179, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n213, new_n214, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n247, new_n248, new_n249, new_n250,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n269, new_n270, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n288,
    new_n289, new_n290, new_n291, new_n292, new_n293, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n346, new_n348, new_n350, new_n351, new_n353,
    new_n354, new_n355, new_n358, new_n359, new_n361, new_n362, new_n363,
    new_n365, new_n366, new_n367;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n09x5               g001(.a(\b[1] ), .b(\a[2] ), .o1(new_n97));
  nand02aa1d28x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n09x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  norb03aa1d15x5               g004(.a(new_n98), .b(new_n97), .c(new_n99), .out0(new_n100));
  oaih12aa1n12x5               g005(.a(new_n98), .b(\b[2] ), .c(\a[3] ), .o1(new_n101));
  nanp02aa1n09x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nand02aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  norp02aa1n12x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanb03aa1d18x5               g009(.a(new_n104), .b(new_n102), .c(new_n103), .out0(new_n105));
  nor002aa1n02x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  aoi012aa1n06x5               g011(.a(new_n104), .b(new_n106), .c(new_n102), .o1(new_n107));
  oai013aa1d12x5               g012(.a(new_n107), .b(new_n100), .c(new_n105), .d(new_n101), .o1(new_n108));
  nor042aa1n02x5               g013(.a(\b[6] ), .b(\a[7] ), .o1(new_n109));
  and002aa1n02x7               g014(.a(\b[6] ), .b(\a[7] ), .o(new_n110));
  nor042aa1n02x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nor002aa1n03x5               g016(.a(\b[4] ), .b(\a[5] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  norb02aa1n02x5               g018(.a(new_n113), .b(new_n112), .out0(new_n114));
  nor002aa1d32x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nand02aa1d06x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb02aa1n09x5               g021(.a(new_n115), .b(new_n116), .out0(new_n117));
  tech160nm_fixorc02aa1n03p5x5 g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  nano32aa1n03x7               g023(.a(new_n117), .b(new_n118), .c(new_n111), .d(new_n114), .out0(new_n119));
  aoi112aa1n03x5               g024(.a(new_n110), .b(new_n109), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  oai012aa1n02x7               g025(.a(new_n116), .b(\b[7] ), .c(\a[8] ), .o1(new_n121));
  oab012aa1n03x5               g026(.a(new_n121), .b(new_n112), .c(new_n115), .out0(new_n122));
  aoi112aa1n03x5               g027(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n123));
  oab012aa1n02x4               g028(.a(new_n123), .b(\a[8] ), .c(\b[7] ), .out0(new_n124));
  aob012aa1n02x5               g029(.a(new_n124), .b(new_n122), .c(new_n120), .out0(new_n125));
  nor042aa1n04x5               g030(.a(\b[8] ), .b(\a[9] ), .o1(new_n126));
  nand42aa1d28x5               g031(.a(\b[8] ), .b(\a[9] ), .o1(new_n127));
  norb02aa1n02x5               g032(.a(new_n127), .b(new_n126), .out0(new_n128));
  aoai13aa1n02x5               g033(.a(new_n128), .b(new_n125), .c(new_n119), .d(new_n108), .o1(new_n129));
  nor042aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  aoib12aa1n02x5               g037(.a(new_n126), .b(new_n131), .c(new_n130), .out0(new_n133));
  oai012aa1n02x5               g038(.a(new_n129), .b(\b[8] ), .c(\a[9] ), .o1(new_n134));
  aoi022aa1n02x5               g039(.a(new_n134), .b(new_n132), .c(new_n129), .d(new_n133), .o1(\s[10] ));
  oai012aa1n02x5               g040(.a(new_n131), .b(new_n130), .c(new_n126), .o1(new_n136));
  nano23aa1n09x5               g041(.a(new_n126), .b(new_n130), .c(new_n131), .d(new_n127), .out0(new_n137));
  aoai13aa1n02x5               g042(.a(new_n137), .b(new_n125), .c(new_n119), .d(new_n108), .o1(new_n138));
  nor042aa1n04x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand42aa1n10x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  norb02aa1n06x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  xnbna2aa1n03x5               g046(.a(new_n141), .b(new_n138), .c(new_n136), .out0(\s[11] ));
  aob012aa1n02x5               g047(.a(new_n141), .b(new_n138), .c(new_n136), .out0(new_n143));
  nor002aa1d24x5               g048(.a(\b[11] ), .b(\a[12] ), .o1(new_n144));
  nand02aa1d06x5               g049(.a(\b[11] ), .b(\a[12] ), .o1(new_n145));
  norb02aa1n06x5               g050(.a(new_n145), .b(new_n144), .out0(new_n146));
  inv000aa1d42x5               g051(.a(new_n144), .o1(new_n147));
  aoi012aa1n02x5               g052(.a(new_n139), .b(new_n147), .c(new_n145), .o1(new_n148));
  oai012aa1n02x5               g053(.a(new_n143), .b(\b[10] ), .c(\a[11] ), .o1(new_n149));
  aoi022aa1n02x5               g054(.a(new_n149), .b(new_n146), .c(new_n143), .d(new_n148), .o1(\s[12] ));
  inv020aa1n03x5               g055(.a(new_n100), .o1(new_n151));
  nona22aa1n09x5               g056(.a(new_n151), .b(new_n105), .c(new_n101), .out0(new_n152));
  nanb02aa1n03x5               g057(.a(new_n112), .b(new_n113), .out0(new_n153));
  nona23aa1n09x5               g058(.a(new_n118), .b(new_n111), .c(new_n153), .d(new_n117), .out0(new_n154));
  aobi12aa1n06x5               g059(.a(new_n124), .b(new_n122), .c(new_n120), .out0(new_n155));
  aoai13aa1n12x5               g060(.a(new_n155), .b(new_n154), .c(new_n152), .d(new_n107), .o1(new_n156));
  nand23aa1d12x5               g061(.a(new_n137), .b(new_n141), .c(new_n146), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  nano22aa1n03x5               g063(.a(new_n144), .b(new_n140), .c(new_n145), .out0(new_n159));
  oai012aa1n02x5               g064(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .o1(new_n160));
  oab012aa1n04x5               g065(.a(new_n160), .b(new_n126), .c(new_n130), .out0(new_n161));
  nanp02aa1n03x5               g066(.a(new_n161), .b(new_n159), .o1(new_n162));
  aoi012aa1n02x7               g067(.a(new_n144), .b(new_n139), .c(new_n145), .o1(new_n163));
  nand22aa1n03x5               g068(.a(new_n162), .b(new_n163), .o1(new_n164));
  nor042aa1n06x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  nand42aa1d28x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n06x5               g072(.a(new_n167), .b(new_n164), .c(new_n156), .d(new_n158), .o1(new_n168));
  inv020aa1n03x5               g073(.a(new_n163), .o1(new_n169));
  nona22aa1n02x4               g074(.a(new_n162), .b(new_n169), .c(new_n167), .out0(new_n170));
  aoi012aa1n02x5               g075(.a(new_n170), .b(new_n156), .c(new_n158), .o1(new_n171));
  norb02aa1n02x5               g076(.a(new_n168), .b(new_n171), .out0(\s[13] ));
  orn002aa1n02x5               g077(.a(\a[13] ), .b(\b[12] ), .o(new_n173));
  nor042aa1n06x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand42aa1d28x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  xnbna2aa1n03x5               g081(.a(new_n176), .b(new_n168), .c(new_n173), .out0(\s[14] ));
  nano23aa1d15x5               g082(.a(new_n165), .b(new_n174), .c(new_n175), .d(new_n166), .out0(new_n178));
  inv000aa1d42x5               g083(.a(new_n178), .o1(new_n179));
  nona22aa1n09x5               g084(.a(new_n156), .b(new_n157), .c(new_n179), .out0(new_n180));
  aoi012aa1d18x5               g085(.a(new_n174), .b(new_n165), .c(new_n175), .o1(new_n181));
  inv000aa1d42x5               g086(.a(new_n181), .o1(new_n182));
  aoi012aa1n02x5               g087(.a(new_n182), .b(new_n164), .c(new_n178), .o1(new_n183));
  xorc02aa1n12x5               g088(.a(\a[15] ), .b(\b[14] ), .out0(new_n184));
  aob012aa1n06x5               g089(.a(new_n184), .b(new_n180), .c(new_n183), .out0(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n182), .c(new_n164), .d(new_n178), .o1(new_n186));
  aobi12aa1n02x5               g091(.a(new_n185), .b(new_n186), .c(new_n180), .out0(\s[15] ));
  xorc02aa1n12x5               g092(.a(\a[16] ), .b(\b[15] ), .out0(new_n188));
  oab012aa1n02x4               g093(.a(new_n188), .b(\a[15] ), .c(\b[14] ), .out0(new_n189));
  tech160nm_fioai012aa1n03p5x5 g094(.a(new_n185), .b(\b[14] ), .c(\a[15] ), .o1(new_n190));
  aoi022aa1n02x5               g095(.a(new_n190), .b(new_n188), .c(new_n185), .d(new_n189), .o1(\s[16] ));
  nano32aa1d15x5               g096(.a(new_n157), .b(new_n188), .c(new_n178), .d(new_n184), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n125), .c(new_n119), .d(new_n108), .o1(new_n193));
  xorc02aa1n03x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  and002aa1n02x5               g099(.a(new_n188), .b(new_n184), .o(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n182), .c(new_n164), .d(new_n178), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n197));
  oab012aa1n02x4               g102(.a(new_n197), .b(\a[16] ), .c(\b[15] ), .out0(new_n198));
  nano22aa1n02x4               g103(.a(new_n194), .b(new_n196), .c(new_n198), .out0(new_n199));
  nanp03aa1d12x5               g104(.a(new_n193), .b(new_n196), .c(new_n198), .o1(new_n200));
  aoi022aa1n02x5               g105(.a(new_n199), .b(new_n193), .c(new_n194), .d(new_n200), .o1(\s[17] ));
  nanp02aa1n02x5               g106(.a(\b[16] ), .b(\a[17] ), .o1(new_n202));
  aoai13aa1n04x5               g107(.a(new_n178), .b(new_n169), .c(new_n161), .d(new_n159), .o1(new_n203));
  inv000aa1n02x5               g108(.a(new_n195), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n198), .b(new_n204), .c(new_n203), .d(new_n181), .o1(new_n205));
  aoai13aa1n02x5               g110(.a(new_n202), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n206));
  inv000aa1d42x5               g111(.a(\a[17] ), .o1(new_n207));
  oaib12aa1n02x5               g112(.a(new_n206), .b(\b[16] ), .c(new_n207), .out0(new_n208));
  xorc02aa1n12x5               g113(.a(\a[18] ), .b(\b[17] ), .out0(new_n209));
  nor002aa1n06x5               g114(.a(\b[16] ), .b(\a[17] ), .o1(new_n210));
  norp02aa1n02x5               g115(.a(new_n209), .b(new_n210), .o1(new_n211));
  aoi022aa1n02x5               g116(.a(new_n208), .b(new_n209), .c(new_n206), .d(new_n211), .o1(\s[18] ));
  inv000aa1d42x5               g117(.a(\a[18] ), .o1(new_n213));
  xroi22aa1d04x5               g118(.a(new_n207), .b(\b[16] ), .c(new_n213), .d(\b[17] ), .out0(new_n214));
  aoai13aa1n06x5               g119(.a(new_n214), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n215));
  inv000aa1d42x5               g120(.a(\b[17] ), .o1(new_n216));
  oao003aa1n02x5               g121(.a(new_n213), .b(new_n216), .c(new_n210), .carry(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  xorc02aa1n12x5               g123(.a(\a[19] ), .b(\b[18] ), .out0(new_n219));
  xnbna2aa1n03x5               g124(.a(new_n219), .b(new_n215), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g125(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g126(.a(new_n219), .b(new_n217), .c(new_n200), .d(new_n214), .o1(new_n222));
  tech160nm_fixorc02aa1n02p5x5 g127(.a(\a[20] ), .b(\b[19] ), .out0(new_n223));
  orn002aa1n02x5               g128(.a(\a[19] ), .b(\b[18] ), .o(new_n224));
  norb02aa1n02x5               g129(.a(new_n224), .b(new_n223), .out0(new_n225));
  inv000aa1d42x5               g130(.a(new_n219), .o1(new_n226));
  aoai13aa1n02x5               g131(.a(new_n224), .b(new_n226), .c(new_n215), .d(new_n218), .o1(new_n227));
  aoi022aa1n03x5               g132(.a(new_n227), .b(new_n223), .c(new_n222), .d(new_n225), .o1(\s[20] ));
  nano32aa1d12x5               g133(.a(new_n226), .b(new_n223), .c(new_n194), .d(new_n209), .out0(new_n229));
  aoai13aa1n06x5               g134(.a(new_n229), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n230));
  nanp02aa1n02x5               g135(.a(\b[19] ), .b(\a[20] ), .o1(new_n231));
  aoi022aa1d24x5               g136(.a(\b[18] ), .b(\a[19] ), .c(\a[18] ), .d(\b[17] ), .o1(new_n232));
  aoai13aa1n04x5               g137(.a(new_n232), .b(new_n210), .c(new_n213), .d(new_n216), .o1(new_n233));
  oai112aa1n06x5               g138(.a(new_n233), .b(new_n224), .c(\b[19] ), .d(\a[20] ), .o1(new_n234));
  and002aa1n02x5               g139(.a(new_n234), .b(new_n231), .o(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  xorc02aa1n12x5               g141(.a(\a[21] ), .b(\b[20] ), .out0(new_n237));
  xnbna2aa1n03x5               g142(.a(new_n237), .b(new_n230), .c(new_n236), .out0(\s[21] ));
  aoai13aa1n03x5               g143(.a(new_n237), .b(new_n235), .c(new_n200), .d(new_n229), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[22] ), .b(\b[21] ), .out0(new_n240));
  nor042aa1n02x5               g145(.a(\b[20] ), .b(\a[21] ), .o1(new_n241));
  norp02aa1n02x5               g146(.a(new_n240), .b(new_n241), .o1(new_n242));
  inv000aa1d42x5               g147(.a(new_n241), .o1(new_n243));
  inv000aa1d42x5               g148(.a(new_n237), .o1(new_n244));
  aoai13aa1n02x5               g149(.a(new_n243), .b(new_n244), .c(new_n230), .d(new_n236), .o1(new_n245));
  aoi022aa1n03x5               g150(.a(new_n245), .b(new_n240), .c(new_n239), .d(new_n242), .o1(\s[22] ));
  and002aa1n02x5               g151(.a(new_n240), .b(new_n237), .o(new_n247));
  and002aa1n02x5               g152(.a(new_n229), .b(new_n247), .o(new_n248));
  aoai13aa1n04x5               g153(.a(new_n248), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n249));
  nand42aa1n08x5               g154(.a(\b[21] ), .b(\a[22] ), .o1(new_n250));
  oai012aa1n02x5               g155(.a(new_n231), .b(\b[20] ), .c(\a[21] ), .o1(new_n251));
  norp02aa1n03x5               g156(.a(\b[21] ), .b(\a[22] ), .o1(new_n252));
  aoi012aa1n02x7               g157(.a(new_n252), .b(\a[21] ), .c(\b[20] ), .o1(new_n253));
  nano32aa1n02x4               g158(.a(new_n251), .b(new_n234), .c(new_n253), .d(new_n250), .out0(new_n254));
  oai012aa1n02x5               g159(.a(new_n250), .b(new_n252), .c(new_n241), .o1(new_n255));
  norb02aa1n02x5               g160(.a(new_n255), .b(new_n254), .out0(new_n256));
  norp02aa1n02x5               g161(.a(\b[22] ), .b(\a[23] ), .o1(new_n257));
  and002aa1n02x5               g162(.a(\b[22] ), .b(\a[23] ), .o(new_n258));
  nor002aa1n02x5               g163(.a(new_n258), .b(new_n257), .o1(new_n259));
  nano22aa1n02x4               g164(.a(new_n254), .b(new_n255), .c(new_n259), .out0(new_n260));
  nand42aa1n03x5               g165(.a(new_n249), .b(new_n260), .o1(new_n261));
  aoai13aa1n02x5               g166(.a(new_n261), .b(new_n259), .c(new_n249), .d(new_n256), .o1(\s[23] ));
  and002aa1n03x5               g167(.a(\b[23] ), .b(\a[24] ), .o(new_n263));
  norp02aa1n04x5               g168(.a(\b[23] ), .b(\a[24] ), .o1(new_n264));
  norp02aa1n02x5               g169(.a(new_n263), .b(new_n264), .o1(new_n265));
  nona22aa1n02x4               g170(.a(new_n261), .b(new_n265), .c(new_n258), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n265), .b(new_n258), .c(new_n249), .d(new_n260), .o1(new_n267));
  nanp02aa1n02x5               g172(.a(new_n266), .b(new_n267), .o1(\s[24] ));
  inv000aa1d42x5               g173(.a(new_n229), .o1(new_n269));
  and002aa1n02x5               g174(.a(new_n265), .b(new_n259), .o(new_n270));
  nano22aa1n02x5               g175(.a(new_n269), .b(new_n247), .c(new_n270), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n271), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n272));
  aoi112aa1n03x5               g177(.a(new_n263), .b(new_n264), .c(\a[23] ), .d(\b[22] ), .o1(new_n273));
  oai012aa1n12x5               g178(.a(new_n250), .b(\b[22] ), .c(\a[23] ), .o1(new_n274));
  norb03aa1n03x5               g179(.a(new_n253), .b(new_n251), .c(new_n274), .out0(new_n275));
  nanp03aa1n06x5               g180(.a(new_n234), .b(new_n275), .c(new_n273), .o1(new_n276));
  oai022aa1n02x5               g181(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n277));
  inv000aa1n02x5               g182(.a(new_n274), .o1(new_n278));
  and003aa1n02x5               g183(.a(new_n273), .b(new_n277), .c(new_n278), .o(new_n279));
  aob012aa1n02x5               g184(.a(new_n257), .b(\b[23] ), .c(\a[24] ), .out0(new_n280));
  nanb02aa1n02x5               g185(.a(new_n264), .b(new_n280), .out0(new_n281));
  nona22aa1d24x5               g186(.a(new_n276), .b(new_n279), .c(new_n281), .out0(new_n282));
  xorc02aa1n12x5               g187(.a(\a[25] ), .b(\b[24] ), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n283), .b(new_n282), .c(new_n200), .d(new_n271), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n283), .o1(new_n285));
  nano23aa1n02x4               g190(.a(new_n281), .b(new_n279), .c(new_n276), .d(new_n285), .out0(new_n286));
  aobi12aa1n02x7               g191(.a(new_n284), .b(new_n286), .c(new_n272), .out0(\s[25] ));
  xorc02aa1n02x5               g192(.a(\a[26] ), .b(\b[25] ), .out0(new_n288));
  nor042aa1n03x5               g193(.a(\b[24] ), .b(\a[25] ), .o1(new_n289));
  norp02aa1n02x5               g194(.a(new_n288), .b(new_n289), .o1(new_n290));
  inv000aa1d42x5               g195(.a(new_n282), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n289), .o1(new_n292));
  aoai13aa1n02x7               g197(.a(new_n292), .b(new_n285), .c(new_n272), .d(new_n291), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n293), .b(new_n288), .c(new_n284), .d(new_n290), .o1(\s[26] ));
  and002aa1n02x5               g199(.a(new_n288), .b(new_n283), .o(new_n295));
  nano32aa1n03x7               g200(.a(new_n269), .b(new_n295), .c(new_n247), .d(new_n270), .out0(new_n296));
  aoai13aa1n06x5               g201(.a(new_n296), .b(new_n205), .c(new_n156), .d(new_n192), .o1(new_n297));
  nor042aa1n04x5               g202(.a(\b[26] ), .b(\a[27] ), .o1(new_n298));
  and002aa1n12x5               g203(.a(\b[26] ), .b(\a[27] ), .o(new_n299));
  norp02aa1n02x5               g204(.a(new_n299), .b(new_n298), .o1(new_n300));
  inv000aa1d42x5               g205(.a(new_n298), .o1(new_n301));
  oao003aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .c(new_n292), .carry(new_n302));
  oaib12aa1n02x5               g207(.a(new_n302), .b(new_n299), .c(new_n301), .out0(new_n303));
  aoi012aa1n02x5               g208(.a(new_n303), .b(new_n282), .c(new_n295), .o1(new_n304));
  aobi12aa1n06x5               g209(.a(new_n302), .b(new_n282), .c(new_n295), .out0(new_n305));
  nanp02aa1n02x5               g210(.a(new_n297), .b(new_n305), .o1(new_n306));
  aoi022aa1n02x5               g211(.a(new_n306), .b(new_n300), .c(new_n297), .d(new_n304), .o1(\s[27] ));
  inv000aa1d42x5               g212(.a(new_n299), .o1(new_n308));
  aob012aa1n02x5               g213(.a(new_n302), .b(new_n282), .c(new_n295), .out0(new_n309));
  aoai13aa1n03x5               g214(.a(new_n308), .b(new_n309), .c(new_n200), .d(new_n296), .o1(new_n310));
  xorc02aa1n02x5               g215(.a(\a[28] ), .b(\b[27] ), .out0(new_n311));
  norp02aa1n02x5               g216(.a(new_n311), .b(new_n298), .o1(new_n312));
  aoai13aa1n06x5               g217(.a(new_n301), .b(new_n299), .c(new_n297), .d(new_n305), .o1(new_n313));
  aoi022aa1n03x5               g218(.a(new_n313), .b(new_n311), .c(new_n310), .d(new_n312), .o1(\s[28] ));
  and002aa1n06x5               g219(.a(new_n311), .b(new_n300), .o(new_n315));
  aoai13aa1n03x5               g220(.a(new_n315), .b(new_n309), .c(new_n200), .d(new_n296), .o1(new_n316));
  inv000aa1n02x5               g221(.a(new_n315), .o1(new_n317));
  oao003aa1n02x5               g222(.a(\a[28] ), .b(\b[27] ), .c(new_n301), .carry(new_n318));
  aoai13aa1n04x5               g223(.a(new_n318), .b(new_n317), .c(new_n297), .d(new_n305), .o1(new_n319));
  xorc02aa1n02x5               g224(.a(\a[29] ), .b(\b[28] ), .out0(new_n320));
  norb02aa1n02x5               g225(.a(new_n318), .b(new_n320), .out0(new_n321));
  aoi022aa1n02x7               g226(.a(new_n319), .b(new_n320), .c(new_n316), .d(new_n321), .o1(\s[29] ));
  xorb03aa1n02x5               g227(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  and003aa1n02x5               g228(.a(new_n311), .b(new_n320), .c(new_n300), .o(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n309), .c(new_n200), .d(new_n296), .o1(new_n325));
  inv000aa1d42x5               g230(.a(new_n324), .o1(new_n326));
  inv000aa1d42x5               g231(.a(\b[28] ), .o1(new_n327));
  inv000aa1d42x5               g232(.a(\a[29] ), .o1(new_n328));
  oaib12aa1n02x5               g233(.a(new_n318), .b(\b[28] ), .c(new_n328), .out0(new_n329));
  oaib12aa1n02x5               g234(.a(new_n329), .b(new_n327), .c(\a[29] ), .out0(new_n330));
  aoai13aa1n02x7               g235(.a(new_n330), .b(new_n326), .c(new_n297), .d(new_n305), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[30] ), .b(\b[29] ), .out0(new_n332));
  oaoi13aa1n02x5               g237(.a(new_n332), .b(new_n329), .c(new_n328), .d(new_n327), .o1(new_n333));
  aoi022aa1n03x5               g238(.a(new_n331), .b(new_n332), .c(new_n325), .d(new_n333), .o1(\s[30] ));
  nanb02aa1n02x5               g239(.a(\b[30] ), .b(\a[31] ), .out0(new_n335));
  nanb02aa1n02x5               g240(.a(\a[31] ), .b(\b[30] ), .out0(new_n336));
  nanp02aa1n02x5               g241(.a(new_n336), .b(new_n335), .o1(new_n337));
  nano22aa1n03x7               g242(.a(new_n317), .b(new_n320), .c(new_n332), .out0(new_n338));
  aoai13aa1n03x5               g243(.a(new_n338), .b(new_n309), .c(new_n200), .d(new_n296), .o1(new_n339));
  inv000aa1d42x5               g244(.a(new_n338), .o1(new_n340));
  norp02aa1n02x5               g245(.a(\b[29] ), .b(\a[30] ), .o1(new_n341));
  aoi022aa1n02x5               g246(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n342));
  aoi012aa1n02x5               g247(.a(new_n341), .b(new_n329), .c(new_n342), .o1(new_n343));
  aoai13aa1n03x5               g248(.a(new_n343), .b(new_n340), .c(new_n297), .d(new_n305), .o1(new_n344));
  oai112aa1n02x5               g249(.a(new_n335), .b(new_n336), .c(\b[29] ), .d(\a[30] ), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n345), .b(new_n329), .c(new_n342), .o1(new_n346));
  aoi022aa1n03x5               g251(.a(new_n344), .b(new_n337), .c(new_n339), .d(new_n346), .o1(\s[31] ));
  norb02aa1n02x5               g252(.a(new_n103), .b(new_n106), .out0(new_n348));
  xobna2aa1n03x5               g253(.a(new_n348), .b(new_n151), .c(new_n98), .out0(\s[3] ));
  aoai13aa1n02x5               g254(.a(new_n348), .b(new_n100), .c(\a[2] ), .d(\b[1] ), .o1(new_n350));
  nanb02aa1n02x5               g255(.a(new_n104), .b(new_n102), .out0(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n350), .c(new_n103), .out0(\s[4] ));
  oai022aa1n02x5               g257(.a(\a[4] ), .b(\b[3] ), .c(\b[4] ), .d(\a[5] ), .o1(new_n353));
  aoi122aa1n02x5               g258(.a(new_n353), .b(\b[4] ), .c(\a[5] ), .d(new_n106), .e(new_n102), .o1(new_n354));
  oai013aa1n03x5               g259(.a(new_n354), .b(new_n100), .c(new_n105), .d(new_n101), .o1(new_n355));
  aoai13aa1n02x5               g260(.a(new_n355), .b(new_n114), .c(new_n152), .d(new_n107), .o1(\s[5] ));
  xnbna2aa1n03x5               g261(.a(new_n117), .b(new_n355), .c(new_n113), .out0(\s[6] ));
  inv000aa1d42x5               g262(.a(new_n115), .o1(new_n358));
  nanb03aa1n02x5               g263(.a(new_n117), .b(new_n355), .c(new_n113), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n111), .b(new_n359), .c(new_n358), .out0(\s[7] ));
  aob012aa1n03x5               g265(.a(new_n111), .b(new_n359), .c(new_n358), .out0(new_n361));
  oai012aa1n02x5               g266(.a(new_n361), .b(\b[6] ), .c(\a[7] ), .o1(new_n362));
  norp02aa1n02x5               g267(.a(new_n118), .b(new_n109), .o1(new_n363));
  aoi022aa1n02x5               g268(.a(new_n362), .b(new_n118), .c(new_n361), .d(new_n363), .o1(\s[8] ));
  nanp02aa1n02x5               g269(.a(new_n119), .b(new_n108), .o1(new_n365));
  oabi12aa1n02x5               g270(.a(new_n123), .b(\a[8] ), .c(\b[7] ), .out0(new_n366));
  aoi112aa1n02x5               g271(.a(new_n366), .b(new_n128), .c(new_n122), .d(new_n120), .o1(new_n367));
  aoi022aa1n02x5               g272(.a(new_n156), .b(new_n128), .c(new_n365), .d(new_n367), .o1(\s[9] ));
endmodule


