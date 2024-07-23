// Benchmark "adder" written by ABC on Thu Jul 18 10:37:37 2024

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
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n175, new_n177,
    new_n178, new_n179, new_n180, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n190, new_n192, new_n193,
    new_n194, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n207, new_n208, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n215, new_n216, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n252, new_n253, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n278, new_n279,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n322, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n331, new_n332, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n344, new_n345, new_n347, new_n349, new_n351, new_n353, new_n354,
    new_n356, new_n357, new_n358, new_n360, new_n362, new_n363;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d30x5               g001(.a(\a[2] ), .o1(new_n97));
  inv040aa1d30x5               g002(.a(\b[1] ), .o1(new_n98));
  nanp02aa1n04x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nand02aa1n03x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nand02aa1d08x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  aob012aa1n03x5               g006(.a(new_n99), .b(new_n100), .c(new_n101), .out0(new_n102));
  xorc02aa1n12x5               g007(.a(\a[4] ), .b(\b[3] ), .out0(new_n103));
  xorc02aa1n12x5               g008(.a(\a[3] ), .b(\b[2] ), .out0(new_n104));
  nanp03aa1n04x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  nor042aa1d18x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  inv040aa1n08x5               g011(.a(new_n106), .o1(new_n107));
  oao003aa1n09x5               g012(.a(\a[4] ), .b(\b[3] ), .c(new_n107), .carry(new_n108));
  nand02aa1n16x5               g013(.a(\b[5] ), .b(\a[6] ), .o1(new_n109));
  nor042aa1n09x5               g014(.a(\b[5] ), .b(\a[6] ), .o1(new_n110));
  norb02aa1n06x5               g015(.a(new_n109), .b(new_n110), .out0(new_n111));
  xnrc02aa1n12x5               g016(.a(\b[4] ), .b(\a[5] ), .out0(new_n112));
  tech160nm_fixnrc02aa1n02p5x5 g017(.a(\b[7] ), .b(\a[8] ), .out0(new_n113));
  nor022aa1n16x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nand42aa1d28x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  norb02aa1n06x4               g020(.a(new_n115), .b(new_n114), .out0(new_n116));
  nona23aa1n09x5               g021(.a(new_n111), .b(new_n116), .c(new_n113), .d(new_n112), .out0(new_n117));
  xorc02aa1n12x5               g022(.a(\a[8] ), .b(\b[7] ), .out0(new_n118));
  nano22aa1n06x5               g023(.a(new_n114), .b(new_n109), .c(new_n115), .out0(new_n119));
  inv000aa1d42x5               g024(.a(\a[5] ), .o1(new_n120));
  inv000aa1d42x5               g025(.a(\b[4] ), .o1(new_n121));
  tech160nm_fiaoi012aa1n05x5   g026(.a(new_n110), .b(new_n120), .c(new_n121), .o1(new_n122));
  inv000aa1n02x5               g027(.a(new_n122), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\a[8] ), .o1(new_n124));
  inv000aa1d42x5               g029(.a(\b[7] ), .o1(new_n125));
  oaoi03aa1n09x5               g030(.a(new_n124), .b(new_n125), .c(new_n114), .o1(new_n126));
  inv000aa1n02x5               g031(.a(new_n126), .o1(new_n127));
  aoi013aa1n06x4               g032(.a(new_n127), .b(new_n119), .c(new_n118), .d(new_n123), .o1(new_n128));
  aoai13aa1n12x5               g033(.a(new_n128), .b(new_n117), .c(new_n105), .d(new_n108), .o1(new_n129));
  nor042aa1n09x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nand42aa1d28x5               g035(.a(\b[8] ), .b(\a[9] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  nor042aa1n09x5               g037(.a(\b[9] ), .b(\a[10] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[9] ), .b(\a[10] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  aoi112aa1n02x5               g040(.a(new_n130), .b(new_n135), .c(new_n129), .d(new_n132), .o1(new_n136));
  aoai13aa1n02x5               g041(.a(new_n135), .b(new_n130), .c(new_n129), .d(new_n131), .o1(new_n137));
  norb02aa1n02x5               g042(.a(new_n137), .b(new_n136), .out0(\s[10] ));
  oaoi03aa1n02x5               g043(.a(new_n97), .b(new_n98), .c(new_n101), .o1(new_n139));
  xnrc02aa1n02x5               g044(.a(\b[3] ), .b(\a[4] ), .out0(new_n140));
  xnrc02aa1n02x5               g045(.a(\b[2] ), .b(\a[3] ), .out0(new_n141));
  oai013aa1n03x5               g046(.a(new_n108), .b(new_n139), .c(new_n140), .d(new_n141), .o1(new_n142));
  nano23aa1n09x5               g047(.a(new_n113), .b(new_n112), .c(new_n116), .d(new_n111), .out0(new_n143));
  nanp03aa1n02x5               g048(.a(new_n119), .b(new_n118), .c(new_n123), .o1(new_n144));
  nanp02aa1n02x5               g049(.a(new_n144), .b(new_n126), .o1(new_n145));
  nano23aa1n06x5               g050(.a(new_n130), .b(new_n133), .c(new_n134), .d(new_n131), .out0(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n145), .c(new_n142), .d(new_n143), .o1(new_n147));
  oai012aa1n02x5               g052(.a(new_n134), .b(new_n133), .c(new_n130), .o1(new_n148));
  xorc02aa1n12x5               g053(.a(\a[11] ), .b(\b[10] ), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n147), .c(new_n148), .out0(\s[11] ));
  aob012aa1n02x5               g055(.a(new_n149), .b(new_n147), .c(new_n148), .out0(new_n151));
  nor042aa1n04x5               g056(.a(\b[11] ), .b(\a[12] ), .o1(new_n152));
  and002aa1n12x5               g057(.a(\b[11] ), .b(\a[12] ), .o(new_n153));
  nor042aa1n06x5               g058(.a(new_n153), .b(new_n152), .o1(new_n154));
  norp02aa1n04x5               g059(.a(\b[10] ), .b(\a[11] ), .o1(new_n155));
  oab012aa1n02x4               g060(.a(new_n155), .b(new_n153), .c(new_n152), .out0(new_n156));
  oai012aa1n02x5               g061(.a(new_n151), .b(\b[10] ), .c(\a[11] ), .o1(new_n157));
  aoi022aa1n02x5               g062(.a(new_n157), .b(new_n154), .c(new_n151), .d(new_n156), .o1(\s[12] ));
  nand03aa1d16x5               g063(.a(new_n146), .b(new_n149), .c(new_n154), .o1(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoi112aa1n03x5               g065(.a(new_n153), .b(new_n152), .c(\a[11] ), .d(\b[10] ), .o1(new_n161));
  oaih12aa1n02x5               g066(.a(new_n134), .b(\b[10] ), .c(\a[11] ), .o1(new_n162));
  oab012aa1n03x5               g067(.a(new_n162), .b(new_n130), .c(new_n133), .out0(new_n163));
  nanp02aa1n03x5               g068(.a(new_n163), .b(new_n161), .o1(new_n164));
  inv000aa1d42x5               g069(.a(\a[12] ), .o1(new_n165));
  inv000aa1d42x5               g070(.a(\b[11] ), .o1(new_n166));
  oaoi03aa1n02x5               g071(.a(new_n165), .b(new_n166), .c(new_n155), .o1(new_n167));
  nand22aa1n03x5               g072(.a(new_n164), .b(new_n167), .o1(new_n168));
  nor042aa1n09x5               g073(.a(\b[12] ), .b(\a[13] ), .o1(new_n169));
  nand42aa1d28x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  norb02aa1n02x5               g075(.a(new_n170), .b(new_n169), .out0(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n168), .c(new_n129), .d(new_n160), .o1(new_n172));
  oao003aa1n02x5               g077(.a(new_n165), .b(new_n166), .c(new_n155), .carry(new_n173));
  nona22aa1n02x4               g078(.a(new_n164), .b(new_n173), .c(new_n171), .out0(new_n174));
  aoi012aa1n02x5               g079(.a(new_n174), .b(new_n129), .c(new_n160), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n172), .b(new_n175), .out0(\s[13] ));
  orn002aa1n02x5               g081(.a(\a[13] ), .b(\b[12] ), .o(new_n177));
  nor042aa1n09x5               g082(.a(\b[13] ), .b(\a[14] ), .o1(new_n178));
  nand42aa1d28x5               g083(.a(\b[13] ), .b(\a[14] ), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n179), .b(new_n178), .out0(new_n180));
  xnbna2aa1n03x5               g085(.a(new_n180), .b(new_n172), .c(new_n177), .out0(\s[14] ));
  nano23aa1d15x5               g086(.a(new_n169), .b(new_n178), .c(new_n179), .d(new_n170), .out0(new_n182));
  inv000aa1d42x5               g087(.a(new_n182), .o1(new_n183));
  nona22aa1n03x5               g088(.a(new_n129), .b(new_n159), .c(new_n183), .out0(new_n184));
  aoi012aa1d18x5               g089(.a(new_n178), .b(new_n169), .c(new_n179), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  tech160nm_fiaoi012aa1n05x5   g091(.a(new_n186), .b(new_n168), .c(new_n182), .o1(new_n187));
  xorc02aa1n12x5               g092(.a(\a[15] ), .b(\b[14] ), .out0(new_n188));
  aob012aa1n03x5               g093(.a(new_n188), .b(new_n184), .c(new_n187), .out0(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n186), .c(new_n168), .d(new_n182), .o1(new_n190));
  aobi12aa1n02x7               g095(.a(new_n189), .b(new_n190), .c(new_n184), .out0(\s[15] ));
  tech160nm_fixorc02aa1n05x5   g096(.a(\a[16] ), .b(\b[15] ), .out0(new_n192));
  oab012aa1n02x4               g097(.a(new_n192), .b(\a[15] ), .c(\b[14] ), .out0(new_n193));
  tech160nm_fioai012aa1n03p5x5 g098(.a(new_n189), .b(\b[14] ), .c(\a[15] ), .o1(new_n194));
  aoi022aa1n02x7               g099(.a(new_n194), .b(new_n192), .c(new_n189), .d(new_n193), .o1(\s[16] ));
  nano32aa1d15x5               g100(.a(new_n159), .b(new_n192), .c(new_n182), .d(new_n188), .out0(new_n196));
  aoai13aa1n06x5               g101(.a(new_n196), .b(new_n145), .c(new_n142), .d(new_n143), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(new_n192), .b(new_n188), .o1(new_n198));
  aoi112aa1n02x5               g103(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n199));
  oab012aa1n02x4               g104(.a(new_n199), .b(\a[16] ), .c(\b[15] ), .out0(new_n200));
  oai112aa1n06x5               g105(.a(new_n197), .b(new_n200), .c(new_n187), .d(new_n198), .o1(new_n201));
  tech160nm_fixorc02aa1n05x5   g106(.a(\a[17] ), .b(\b[16] ), .out0(new_n202));
  aoai13aa1n04x5               g107(.a(new_n182), .b(new_n173), .c(new_n163), .d(new_n161), .o1(new_n203));
  aoi012aa1n02x5               g108(.a(new_n198), .b(new_n203), .c(new_n185), .o1(new_n204));
  norb03aa1n02x5               g109(.a(new_n200), .b(new_n204), .c(new_n202), .out0(new_n205));
  aoi022aa1n02x5               g110(.a(new_n201), .b(new_n202), .c(new_n205), .d(new_n197), .o1(\s[17] ));
  nor002aa1d32x5               g111(.a(\b[16] ), .b(\a[17] ), .o1(new_n207));
  inv000aa1d42x5               g112(.a(new_n207), .o1(new_n208));
  aoai13aa1n06x5               g113(.a(new_n200), .b(new_n198), .c(new_n203), .d(new_n185), .o1(new_n209));
  aoai13aa1n03x5               g114(.a(new_n202), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n210));
  nor042aa1n06x5               g115(.a(\b[17] ), .b(\a[18] ), .o1(new_n211));
  nand42aa1n10x5               g116(.a(\b[17] ), .b(\a[18] ), .o1(new_n212));
  norb02aa1n06x4               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n210), .c(new_n208), .out0(\s[18] ));
  and002aa1n02x5               g119(.a(new_n202), .b(new_n213), .o(new_n215));
  aoai13aa1n03x5               g120(.a(new_n215), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n216));
  oaoi03aa1n02x5               g121(.a(\a[18] ), .b(\b[17] ), .c(new_n208), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n217), .o1(new_n218));
  nor002aa1d32x5               g123(.a(\b[18] ), .b(\a[19] ), .o1(new_n219));
  nand42aa1n10x5               g124(.a(\b[18] ), .b(\a[19] ), .o1(new_n220));
  norb02aa1n12x5               g125(.a(new_n220), .b(new_n219), .out0(new_n221));
  xnbna2aa1n03x5               g126(.a(new_n221), .b(new_n216), .c(new_n218), .out0(\s[19] ));
  xnrc02aa1n02x5               g127(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g128(.a(new_n221), .b(new_n217), .c(new_n201), .d(new_n215), .o1(new_n224));
  nor002aa1d32x5               g129(.a(\b[19] ), .b(\a[20] ), .o1(new_n225));
  nand02aa1d28x5               g130(.a(\b[19] ), .b(\a[20] ), .o1(new_n226));
  norb02aa1n02x5               g131(.a(new_n226), .b(new_n225), .out0(new_n227));
  inv000aa1d42x5               g132(.a(\a[19] ), .o1(new_n228));
  inv000aa1d42x5               g133(.a(\b[18] ), .o1(new_n229));
  aboi22aa1n03x5               g134(.a(new_n225), .b(new_n226), .c(new_n228), .d(new_n229), .out0(new_n230));
  inv000aa1n04x5               g135(.a(new_n219), .o1(new_n231));
  inv000aa1d42x5               g136(.a(new_n221), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n231), .b(new_n232), .c(new_n216), .d(new_n218), .o1(new_n233));
  aoi022aa1n02x7               g138(.a(new_n233), .b(new_n227), .c(new_n224), .d(new_n230), .o1(\s[20] ));
  nano32aa1n03x7               g139(.a(new_n232), .b(new_n202), .c(new_n227), .d(new_n213), .out0(new_n235));
  aoai13aa1n03x5               g140(.a(new_n235), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n236));
  nanb03aa1n09x5               g141(.a(new_n225), .b(new_n226), .c(new_n220), .out0(new_n237));
  oai112aa1n06x5               g142(.a(new_n231), .b(new_n212), .c(new_n211), .d(new_n207), .o1(new_n238));
  aoi012aa1d24x5               g143(.a(new_n225), .b(new_n219), .c(new_n226), .o1(new_n239));
  oai012aa1d24x5               g144(.a(new_n239), .b(new_n238), .c(new_n237), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nanp02aa1n02x5               g146(.a(new_n236), .b(new_n241), .o1(new_n242));
  nor042aa1d18x5               g147(.a(\b[20] ), .b(\a[21] ), .o1(new_n243));
  nand02aa1n06x5               g148(.a(\b[20] ), .b(\a[21] ), .o1(new_n244));
  norb02aa1n12x5               g149(.a(new_n244), .b(new_n243), .out0(new_n245));
  nano22aa1n02x5               g150(.a(new_n225), .b(new_n220), .c(new_n226), .out0(new_n246));
  tech160nm_fioai012aa1n03p5x5 g151(.a(new_n212), .b(\b[18] ), .c(\a[19] ), .o1(new_n247));
  oab012aa1n02x5               g152(.a(new_n247), .b(new_n207), .c(new_n211), .out0(new_n248));
  inv000aa1n02x5               g153(.a(new_n239), .o1(new_n249));
  aoi112aa1n02x5               g154(.a(new_n249), .b(new_n245), .c(new_n248), .d(new_n246), .o1(new_n250));
  aoi022aa1n02x7               g155(.a(new_n242), .b(new_n245), .c(new_n236), .d(new_n250), .o1(\s[21] ));
  aoai13aa1n03x5               g156(.a(new_n245), .b(new_n240), .c(new_n201), .d(new_n235), .o1(new_n252));
  nor042aa1n06x5               g157(.a(\b[21] ), .b(\a[22] ), .o1(new_n253));
  nand42aa1n16x5               g158(.a(\b[21] ), .b(\a[22] ), .o1(new_n254));
  norb02aa1n02x5               g159(.a(new_n254), .b(new_n253), .out0(new_n255));
  aoib12aa1n02x5               g160(.a(new_n243), .b(new_n254), .c(new_n253), .out0(new_n256));
  inv000aa1d42x5               g161(.a(new_n243), .o1(new_n257));
  inv000aa1d42x5               g162(.a(new_n245), .o1(new_n258));
  aoai13aa1n04x5               g163(.a(new_n257), .b(new_n258), .c(new_n236), .d(new_n241), .o1(new_n259));
  aoi022aa1n02x7               g164(.a(new_n259), .b(new_n255), .c(new_n252), .d(new_n256), .o1(\s[22] ));
  inv020aa1n03x5               g165(.a(new_n235), .o1(new_n261));
  nano22aa1n03x7               g166(.a(new_n261), .b(new_n245), .c(new_n255), .out0(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n263));
  nano23aa1d15x5               g168(.a(new_n243), .b(new_n253), .c(new_n254), .d(new_n244), .out0(new_n264));
  aoi012aa1d18x5               g169(.a(new_n253), .b(new_n243), .c(new_n254), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoi012aa1n09x5               g171(.a(new_n266), .b(new_n240), .c(new_n264), .o1(new_n267));
  nand42aa1n02x5               g172(.a(new_n263), .b(new_n267), .o1(new_n268));
  xorc02aa1n12x5               g173(.a(\a[23] ), .b(\b[22] ), .out0(new_n269));
  aoi112aa1n02x5               g174(.a(new_n269), .b(new_n266), .c(new_n240), .d(new_n264), .o1(new_n270));
  aoi022aa1n02x7               g175(.a(new_n268), .b(new_n269), .c(new_n263), .d(new_n270), .o1(\s[23] ));
  inv000aa1d42x5               g176(.a(new_n267), .o1(new_n272));
  aoai13aa1n03x5               g177(.a(new_n269), .b(new_n272), .c(new_n201), .d(new_n262), .o1(new_n273));
  xorc02aa1n03x5               g178(.a(\a[24] ), .b(\b[23] ), .out0(new_n274));
  nor002aa1d32x5               g179(.a(\b[22] ), .b(\a[23] ), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n274), .b(new_n275), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n275), .o1(new_n277));
  inv000aa1d42x5               g182(.a(new_n269), .o1(new_n278));
  aoai13aa1n02x7               g183(.a(new_n277), .b(new_n278), .c(new_n263), .d(new_n267), .o1(new_n279));
  aoi022aa1n02x7               g184(.a(new_n279), .b(new_n274), .c(new_n273), .d(new_n276), .o1(\s[24] ));
  and002aa1n12x5               g185(.a(new_n274), .b(new_n269), .o(new_n281));
  nano22aa1n02x5               g186(.a(new_n261), .b(new_n281), .c(new_n264), .out0(new_n282));
  aoai13aa1n03x5               g187(.a(new_n282), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n283));
  aoai13aa1n04x5               g188(.a(new_n264), .b(new_n249), .c(new_n248), .d(new_n246), .o1(new_n284));
  inv000aa1n09x5               g189(.a(new_n281), .o1(new_n285));
  oao003aa1n02x5               g190(.a(\a[24] ), .b(\b[23] ), .c(new_n277), .carry(new_n286));
  aoai13aa1n12x5               g191(.a(new_n286), .b(new_n285), .c(new_n284), .d(new_n265), .o1(new_n287));
  inv000aa1d42x5               g192(.a(new_n287), .o1(new_n288));
  nanp02aa1n02x5               g193(.a(new_n283), .b(new_n288), .o1(new_n289));
  xorc02aa1n12x5               g194(.a(\a[25] ), .b(\b[24] ), .out0(new_n290));
  aoai13aa1n02x5               g195(.a(new_n281), .b(new_n266), .c(new_n240), .d(new_n264), .o1(new_n291));
  inv000aa1d42x5               g196(.a(new_n290), .o1(new_n292));
  and003aa1n02x5               g197(.a(new_n291), .b(new_n292), .c(new_n286), .o(new_n293));
  aoi022aa1n02x7               g198(.a(new_n289), .b(new_n290), .c(new_n283), .d(new_n293), .o1(\s[25] ));
  aoai13aa1n03x5               g199(.a(new_n290), .b(new_n287), .c(new_n201), .d(new_n282), .o1(new_n295));
  tech160nm_fixorc02aa1n03p5x5 g200(.a(\a[26] ), .b(\b[25] ), .out0(new_n296));
  nor042aa1n06x5               g201(.a(\b[24] ), .b(\a[25] ), .o1(new_n297));
  norp02aa1n02x5               g202(.a(new_n296), .b(new_n297), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n297), .o1(new_n299));
  aoai13aa1n04x5               g204(.a(new_n299), .b(new_n292), .c(new_n283), .d(new_n288), .o1(new_n300));
  aoi022aa1n02x7               g205(.a(new_n300), .b(new_n296), .c(new_n295), .d(new_n298), .o1(\s[26] ));
  and002aa1n18x5               g206(.a(new_n296), .b(new_n290), .o(new_n302));
  nano32aa1n03x7               g207(.a(new_n261), .b(new_n302), .c(new_n264), .d(new_n281), .out0(new_n303));
  aoai13aa1n06x5               g208(.a(new_n303), .b(new_n209), .c(new_n129), .d(new_n196), .o1(new_n304));
  oao003aa1n06x5               g209(.a(\a[26] ), .b(\b[25] ), .c(new_n299), .carry(new_n305));
  inv000aa1d42x5               g210(.a(new_n305), .o1(new_n306));
  aoi012aa1n09x5               g211(.a(new_n306), .b(new_n287), .c(new_n302), .o1(new_n307));
  nand42aa1n02x5               g212(.a(new_n304), .b(new_n307), .o1(new_n308));
  xorc02aa1n12x5               g213(.a(\a[27] ), .b(\b[26] ), .out0(new_n309));
  aoi112aa1n02x5               g214(.a(new_n309), .b(new_n306), .c(new_n287), .d(new_n302), .o1(new_n310));
  aoi022aa1n02x7               g215(.a(new_n308), .b(new_n309), .c(new_n304), .d(new_n310), .o1(\s[27] ));
  inv000aa1d42x5               g216(.a(new_n302), .o1(new_n312));
  aoai13aa1n04x5               g217(.a(new_n305), .b(new_n312), .c(new_n291), .d(new_n286), .o1(new_n313));
  aoai13aa1n03x5               g218(.a(new_n309), .b(new_n313), .c(new_n201), .d(new_n303), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[28] ), .b(\b[27] ), .out0(new_n315));
  norp02aa1n02x5               g220(.a(\b[26] ), .b(\a[27] ), .o1(new_n316));
  norp02aa1n02x5               g221(.a(new_n315), .b(new_n316), .o1(new_n317));
  inv000aa1n03x5               g222(.a(new_n316), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n309), .o1(new_n319));
  aoai13aa1n02x7               g224(.a(new_n318), .b(new_n319), .c(new_n304), .d(new_n307), .o1(new_n320));
  aoi022aa1n03x5               g225(.a(new_n320), .b(new_n315), .c(new_n314), .d(new_n317), .o1(\s[28] ));
  and002aa1n02x5               g226(.a(new_n315), .b(new_n309), .o(new_n322));
  aoai13aa1n03x5               g227(.a(new_n322), .b(new_n313), .c(new_n201), .d(new_n303), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[29] ), .b(\b[28] ), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[28] ), .b(\b[27] ), .c(new_n318), .carry(new_n325));
  norb02aa1n02x5               g230(.a(new_n325), .b(new_n324), .out0(new_n326));
  inv000aa1d42x5               g231(.a(new_n322), .o1(new_n327));
  aoai13aa1n02x7               g232(.a(new_n325), .b(new_n327), .c(new_n304), .d(new_n307), .o1(new_n328));
  aoi022aa1n02x7               g233(.a(new_n328), .b(new_n324), .c(new_n323), .d(new_n326), .o1(\s[29] ));
  xorb03aa1n02x5               g234(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n06x5               g235(.a(new_n319), .b(new_n315), .c(new_n324), .out0(new_n331));
  aoai13aa1n03x5               g236(.a(new_n331), .b(new_n313), .c(new_n201), .d(new_n303), .o1(new_n332));
  xorc02aa1n02x5               g237(.a(\a[30] ), .b(\b[29] ), .out0(new_n333));
  oao003aa1n02x5               g238(.a(\a[29] ), .b(\b[28] ), .c(new_n325), .carry(new_n334));
  norb02aa1n02x5               g239(.a(new_n334), .b(new_n333), .out0(new_n335));
  inv000aa1d42x5               g240(.a(new_n331), .o1(new_n336));
  aoai13aa1n02x7               g241(.a(new_n334), .b(new_n336), .c(new_n304), .d(new_n307), .o1(new_n337));
  aoi022aa1n03x5               g242(.a(new_n337), .b(new_n333), .c(new_n332), .d(new_n335), .o1(\s[30] ));
  nano32aa1n06x5               g243(.a(new_n319), .b(new_n333), .c(new_n315), .d(new_n324), .out0(new_n339));
  aoai13aa1n03x5               g244(.a(new_n339), .b(new_n313), .c(new_n201), .d(new_n303), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[31] ), .b(\b[30] ), .out0(new_n341));
  oao003aa1n02x5               g246(.a(\a[30] ), .b(\b[29] ), .c(new_n334), .carry(new_n342));
  norb02aa1n02x5               g247(.a(new_n342), .b(new_n341), .out0(new_n343));
  inv000aa1d42x5               g248(.a(new_n339), .o1(new_n344));
  aoai13aa1n02x7               g249(.a(new_n342), .b(new_n344), .c(new_n304), .d(new_n307), .o1(new_n345));
  aoi022aa1n03x5               g250(.a(new_n345), .b(new_n341), .c(new_n340), .d(new_n343), .o1(\s[31] ));
  nanp03aa1n02x5               g251(.a(new_n99), .b(new_n100), .c(new_n101), .o1(new_n347));
  xnbna2aa1n03x5               g252(.a(new_n104), .b(new_n347), .c(new_n99), .out0(\s[3] ));
  nanp02aa1n02x5               g253(.a(new_n102), .b(new_n104), .o1(new_n349));
  xnbna2aa1n03x5               g254(.a(new_n103), .b(new_n349), .c(new_n107), .out0(\s[4] ));
  inv000aa1d42x5               g255(.a(new_n112), .o1(new_n351));
  xnbna2aa1n03x5               g256(.a(new_n351), .b(new_n105), .c(new_n108), .out0(\s[5] ));
  nanp02aa1n02x5               g257(.a(new_n121), .b(new_n120), .o1(new_n353));
  nanp02aa1n02x5               g258(.a(new_n142), .b(new_n351), .o1(new_n354));
  xnbna2aa1n03x5               g259(.a(new_n111), .b(new_n354), .c(new_n353), .out0(\s[6] ));
  aoai13aa1n02x5               g260(.a(new_n353), .b(new_n112), .c(new_n105), .d(new_n108), .o1(new_n356));
  aoai13aa1n03x5               g261(.a(new_n116), .b(new_n110), .c(new_n356), .d(new_n109), .o1(new_n357));
  aoi112aa1n02x5               g262(.a(new_n116), .b(new_n110), .c(new_n356), .d(new_n111), .o1(new_n358));
  norb02aa1n02x5               g263(.a(new_n357), .b(new_n358), .out0(\s[7] ));
  orn002aa1n02x5               g264(.a(\a[7] ), .b(\b[6] ), .o(new_n360));
  xnbna2aa1n03x5               g265(.a(new_n118), .b(new_n357), .c(new_n360), .out0(\s[8] ));
  nanp02aa1n02x5               g266(.a(new_n142), .b(new_n143), .o1(new_n362));
  aoi113aa1n02x5               g267(.a(new_n127), .b(new_n132), .c(new_n118), .d(new_n119), .e(new_n123), .o1(new_n363));
  aoi022aa1n02x5               g268(.a(new_n129), .b(new_n132), .c(new_n362), .d(new_n363), .o1(\s[9] ));
endmodule


