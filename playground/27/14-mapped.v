// Benchmark "adder" written by ABC on Thu Jul 18 01:53:56 2024

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
    new_n132, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n187, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n200, new_n201, new_n202,
    new_n203, new_n204, new_n205, new_n206, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n214, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n227, new_n228, new_n229, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n239, new_n240, new_n241, new_n242,
    new_n243, new_n244, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n251, new_n252, new_n253, new_n254, new_n255, new_n256, new_n258,
    new_n259, new_n260, new_n261, new_n262, new_n263, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n270, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n302, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n312, new_n313, new_n314, new_n315, new_n317, new_n318, new_n319,
    new_n320, new_n321, new_n322, new_n323, new_n324, new_n327, new_n328,
    new_n329, new_n330, new_n331, new_n332, new_n333, new_n334, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n343,
    new_n346, new_n349, new_n350, new_n352, new_n354;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  nor002aa1n02x5               g002(.a(\b[8] ), .b(\a[9] ), .o1(new_n98));
  nanp02aa1n09x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(new_n99), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[0] ), .b(\a[1] ), .o1(new_n101));
  nor042aa1n02x5               g006(.a(\b[1] ), .b(\a[2] ), .o1(new_n102));
  oab012aa1n02x4               g007(.a(new_n100), .b(new_n102), .c(new_n101), .out0(new_n103));
  nor042aa1n02x5               g008(.a(\b[3] ), .b(\a[4] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  norb02aa1n02x5               g010(.a(new_n105), .b(new_n104), .out0(new_n106));
  norp02aa1n02x5               g011(.a(\b[2] ), .b(\a[3] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[2] ), .b(\a[3] ), .o1(new_n108));
  norb02aa1n02x5               g013(.a(new_n108), .b(new_n107), .out0(new_n109));
  nanp03aa1n02x5               g014(.a(new_n103), .b(new_n106), .c(new_n109), .o1(new_n110));
  inv000aa1d42x5               g015(.a(\a[3] ), .o1(new_n111));
  inv000aa1d42x5               g016(.a(\b[2] ), .o1(new_n112));
  nand02aa1d08x5               g017(.a(new_n112), .b(new_n111), .o1(new_n113));
  oaoi03aa1n02x5               g018(.a(\a[4] ), .b(\b[3] ), .c(new_n113), .o1(new_n114));
  inv040aa1n02x5               g019(.a(new_n114), .o1(new_n115));
  nor042aa1n04x5               g020(.a(\b[7] ), .b(\a[8] ), .o1(new_n116));
  tech160nm_finand02aa1n05x5   g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  norp02aa1n12x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nand42aa1n03x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano23aa1n02x4               g024(.a(new_n116), .b(new_n118), .c(new_n119), .d(new_n117), .out0(new_n120));
  xorc02aa1n02x5               g025(.a(\a[6] ), .b(\b[5] ), .out0(new_n121));
  xorc02aa1n02x5               g026(.a(\a[5] ), .b(\b[4] ), .out0(new_n122));
  nanp03aa1n02x5               g027(.a(new_n120), .b(new_n121), .c(new_n122), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\a[6] ), .o1(new_n124));
  nor042aa1n03x5               g029(.a(\b[4] ), .b(\a[5] ), .o1(new_n125));
  aob012aa1n02x5               g030(.a(new_n125), .b(\b[5] ), .c(\a[6] ), .out0(new_n126));
  oaib12aa1n02x5               g031(.a(new_n126), .b(\b[5] ), .c(new_n124), .out0(new_n127));
  aoi012aa1n02x5               g032(.a(new_n116), .b(new_n118), .c(new_n117), .o1(new_n128));
  aobi12aa1n06x5               g033(.a(new_n128), .b(new_n120), .c(new_n127), .out0(new_n129));
  aoai13aa1n04x5               g034(.a(new_n129), .b(new_n123), .c(new_n110), .d(new_n115), .o1(new_n130));
  xnrc02aa1n02x5               g035(.a(\b[8] ), .b(\a[9] ), .out0(new_n131));
  aoib12aa1n02x5               g036(.a(new_n98), .b(new_n130), .c(new_n131), .out0(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  xnrc02aa1n02x5               g038(.a(\b[9] ), .b(\a[10] ), .out0(new_n134));
  norp02aa1n02x5               g039(.a(new_n131), .b(new_n134), .o1(new_n135));
  inv000aa1d42x5               g040(.a(\b[9] ), .o1(new_n136));
  oao003aa1n02x5               g041(.a(new_n97), .b(new_n136), .c(new_n98), .carry(new_n137));
  aoi012aa1n03x5               g042(.a(new_n137), .b(new_n130), .c(new_n135), .o1(new_n138));
  xnrb03aa1n02x5               g043(.a(new_n138), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g044(.a(\a[11] ), .b(\b[10] ), .c(new_n138), .o1(new_n140));
  xorb03aa1n02x5               g045(.a(new_n140), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  oai012aa1n12x5               g046(.a(new_n99), .b(new_n102), .c(new_n101), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n104), .b(new_n105), .out0(new_n143));
  nanp02aa1n02x5               g048(.a(new_n113), .b(new_n108), .o1(new_n144));
  norp03aa1n04x5               g049(.a(new_n142), .b(new_n143), .c(new_n144), .o1(new_n145));
  nona23aa1n02x4               g050(.a(new_n119), .b(new_n117), .c(new_n116), .d(new_n118), .out0(new_n146));
  xnrc02aa1n02x5               g051(.a(\b[5] ), .b(\a[6] ), .out0(new_n147));
  xnrc02aa1n02x5               g052(.a(\b[4] ), .b(\a[5] ), .out0(new_n148));
  nor043aa1n02x5               g053(.a(new_n146), .b(new_n147), .c(new_n148), .o1(new_n149));
  oai012aa1n06x5               g054(.a(new_n149), .b(new_n145), .c(new_n114), .o1(new_n150));
  nor002aa1n03x5               g055(.a(\b[10] ), .b(\a[11] ), .o1(new_n151));
  nanp02aa1n02x5               g056(.a(\b[10] ), .b(\a[11] ), .o1(new_n152));
  nor042aa1n02x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nand22aa1n02x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  nano23aa1n03x5               g059(.a(new_n151), .b(new_n153), .c(new_n154), .d(new_n152), .out0(new_n155));
  nona22aa1n02x4               g060(.a(new_n155), .b(new_n131), .c(new_n134), .out0(new_n156));
  aoi012aa1n02x5               g061(.a(new_n153), .b(new_n151), .c(new_n154), .o1(new_n157));
  aobi12aa1n02x5               g062(.a(new_n157), .b(new_n155), .c(new_n137), .out0(new_n158));
  aoai13aa1n03x5               g063(.a(new_n158), .b(new_n156), .c(new_n150), .d(new_n129), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  inv030aa1d32x5               g065(.a(\a[13] ), .o1(new_n161));
  inv000aa1n30x5               g066(.a(\b[12] ), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(new_n161), .b(new_n162), .c(new_n159), .o1(new_n163));
  xnrb03aa1n03x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  oai013aa1n03x5               g069(.a(new_n115), .b(new_n143), .c(new_n142), .d(new_n144), .o1(new_n165));
  oaib12aa1n02x5               g070(.a(new_n128), .b(new_n146), .c(new_n127), .out0(new_n166));
  nona23aa1n03x5               g071(.a(new_n154), .b(new_n152), .c(new_n151), .d(new_n153), .out0(new_n167));
  norp03aa1n02x5               g072(.a(new_n167), .b(new_n131), .c(new_n134), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n168), .b(new_n166), .c(new_n165), .d(new_n149), .o1(new_n169));
  nor022aa1n04x5               g074(.a(\b[12] ), .b(\a[13] ), .o1(new_n170));
  nanp02aa1n02x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  nor002aa1d32x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n08x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n02x4               g078(.a(new_n173), .b(new_n171), .c(new_n170), .d(new_n172), .out0(new_n174));
  aoai13aa1n12x5               g079(.a(new_n173), .b(new_n172), .c(new_n161), .d(new_n162), .o1(new_n175));
  aoai13aa1n03x5               g080(.a(new_n175), .b(new_n174), .c(new_n169), .d(new_n158), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1d18x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  tech160nm_fixorc02aa1n02p5x5 g083(.a(\a[15] ), .b(\b[14] ), .out0(new_n179));
  xorc02aa1n02x5               g084(.a(\a[16] ), .b(\b[15] ), .out0(new_n180));
  aoi112aa1n02x5               g085(.a(new_n180), .b(new_n178), .c(new_n176), .d(new_n179), .o1(new_n181));
  inv040aa1n08x5               g086(.a(new_n178), .o1(new_n182));
  nano23aa1n06x5               g087(.a(new_n170), .b(new_n172), .c(new_n173), .d(new_n171), .out0(new_n183));
  inv000aa1d42x5               g088(.a(new_n175), .o1(new_n184));
  aoai13aa1n03x5               g089(.a(new_n179), .b(new_n184), .c(new_n159), .d(new_n183), .o1(new_n185));
  xnrc02aa1n03x5               g090(.a(\b[15] ), .b(\a[16] ), .out0(new_n186));
  tech160nm_fiaoi012aa1n02p5x5 g091(.a(new_n186), .b(new_n185), .c(new_n182), .o1(new_n187));
  norp02aa1n03x5               g092(.a(new_n187), .b(new_n181), .o1(\s[16] ));
  tech160nm_fixnrc02aa1n02p5x5 g093(.a(\b[14] ), .b(\a[15] ), .out0(new_n189));
  nor043aa1n02x5               g094(.a(new_n174), .b(new_n186), .c(new_n189), .o1(new_n190));
  nanp02aa1n02x5               g095(.a(new_n168), .b(new_n190), .o1(new_n191));
  oaoi03aa1n02x5               g096(.a(new_n97), .b(new_n136), .c(new_n98), .o1(new_n192));
  tech160nm_fioai012aa1n03p5x5 g097(.a(new_n157), .b(new_n167), .c(new_n192), .o1(new_n193));
  oaoi03aa1n09x5               g098(.a(\a[16] ), .b(\b[15] ), .c(new_n182), .o1(new_n194));
  inv040aa1d30x5               g099(.a(new_n194), .o1(new_n195));
  oai013aa1n03x5               g100(.a(new_n195), .b(new_n186), .c(new_n189), .d(new_n175), .o1(new_n196));
  aoi012aa1n12x5               g101(.a(new_n196), .b(new_n193), .c(new_n190), .o1(new_n197));
  aoai13aa1n12x5               g102(.a(new_n197), .b(new_n191), .c(new_n150), .d(new_n129), .o1(new_n198));
  xorb03aa1n02x5               g103(.a(new_n198), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  nanp03aa1n02x5               g104(.a(new_n183), .b(new_n179), .c(new_n180), .o1(new_n200));
  nor042aa1n02x5               g105(.a(new_n156), .b(new_n200), .o1(new_n201));
  aoai13aa1n06x5               g106(.a(new_n201), .b(new_n166), .c(new_n165), .d(new_n149), .o1(new_n202));
  nor002aa1d32x5               g107(.a(\b[16] ), .b(\a[17] ), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  and002aa1n02x5               g109(.a(\b[16] ), .b(\a[17] ), .o(new_n205));
  aoi013aa1n03x5               g110(.a(new_n205), .b(new_n202), .c(new_n197), .d(new_n204), .o1(new_n206));
  xorb03aa1n02x5               g111(.a(new_n206), .b(\b[17] ), .c(\a[18] ), .out0(\s[18] ));
  inv000aa1d42x5               g112(.a(\a[17] ), .o1(new_n208));
  inv040aa1d32x5               g113(.a(\a[18] ), .o1(new_n209));
  xroi22aa1d06x4               g114(.a(new_n208), .b(\b[16] ), .c(new_n209), .d(\b[17] ), .out0(new_n210));
  inv000aa1d42x5               g115(.a(new_n210), .o1(new_n211));
  inv000aa1d42x5               g116(.a(\b[17] ), .o1(new_n212));
  oaoi03aa1n12x5               g117(.a(new_n209), .b(new_n212), .c(new_n203), .o1(new_n213));
  aoai13aa1n04x5               g118(.a(new_n213), .b(new_n211), .c(new_n202), .d(new_n197), .o1(new_n214));
  xorb03aa1n02x5               g119(.a(new_n214), .b(\b[18] ), .c(\a[19] ), .out0(\s[19] ));
  xnrc02aa1n02x5               g120(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n16x5               g121(.a(\b[18] ), .b(\a[19] ), .o1(new_n217));
  tech160nm_finand02aa1n03p5x5 g122(.a(\b[18] ), .b(\a[19] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(new_n217), .b(new_n218), .out0(new_n219));
  inv000aa1d42x5               g124(.a(new_n219), .o1(new_n220));
  nor022aa1n08x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  tech160nm_finand02aa1n03p5x5 g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  nanb02aa1n02x5               g127(.a(new_n221), .b(new_n222), .out0(new_n223));
  inv000aa1d42x5               g128(.a(new_n223), .o1(new_n224));
  aoi112aa1n02x5               g129(.a(new_n217), .b(new_n224), .c(new_n214), .d(new_n220), .o1(new_n225));
  inv000aa1n02x5               g130(.a(new_n217), .o1(new_n226));
  inv000aa1d42x5               g131(.a(new_n213), .o1(new_n227));
  aoai13aa1n03x5               g132(.a(new_n220), .b(new_n227), .c(new_n198), .d(new_n210), .o1(new_n228));
  aoi012aa1n03x5               g133(.a(new_n223), .b(new_n228), .c(new_n226), .o1(new_n229));
  norp02aa1n03x5               g134(.a(new_n229), .b(new_n225), .o1(\s[20] ));
  nona23aa1d18x5               g135(.a(new_n222), .b(new_n218), .c(new_n217), .d(new_n221), .out0(new_n231));
  norb02aa1n02x5               g136(.a(new_n210), .b(new_n231), .out0(new_n232));
  inv000aa1n02x5               g137(.a(new_n232), .o1(new_n233));
  oaoi03aa1n02x5               g138(.a(\a[20] ), .b(\b[19] ), .c(new_n226), .o1(new_n234));
  oabi12aa1n18x5               g139(.a(new_n234), .b(new_n231), .c(new_n213), .out0(new_n235));
  inv000aa1d42x5               g140(.a(new_n235), .o1(new_n236));
  aoai13aa1n04x5               g141(.a(new_n236), .b(new_n233), .c(new_n202), .d(new_n197), .o1(new_n237));
  xorb03aa1n02x5               g142(.a(new_n237), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g143(.a(\b[20] ), .b(\a[21] ), .o1(new_n239));
  xnrc02aa1n12x5               g144(.a(\b[20] ), .b(\a[21] ), .out0(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nor022aa1n16x5               g146(.a(\b[21] ), .b(\a[22] ), .o1(new_n242));
  nand42aa1n08x5               g147(.a(\b[21] ), .b(\a[22] ), .o1(new_n243));
  nanb02aa1d24x5               g148(.a(new_n242), .b(new_n243), .out0(new_n244));
  inv000aa1d42x5               g149(.a(new_n244), .o1(new_n245));
  aoi112aa1n02x5               g150(.a(new_n239), .b(new_n245), .c(new_n237), .d(new_n241), .o1(new_n246));
  inv000aa1d42x5               g151(.a(new_n239), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n241), .b(new_n235), .c(new_n198), .d(new_n232), .o1(new_n248));
  aoi012aa1n03x5               g153(.a(new_n244), .b(new_n248), .c(new_n247), .o1(new_n249));
  nor002aa1n02x5               g154(.a(new_n249), .b(new_n246), .o1(\s[22] ));
  inv000aa1d42x5               g155(.a(new_n231), .o1(new_n251));
  nor042aa1n04x5               g156(.a(new_n240), .b(new_n244), .o1(new_n252));
  nand23aa1d12x5               g157(.a(new_n210), .b(new_n251), .c(new_n252), .o1(new_n253));
  oai012aa1n02x5               g158(.a(new_n243), .b(new_n242), .c(new_n239), .o1(new_n254));
  aobi12aa1n12x5               g159(.a(new_n254), .b(new_n235), .c(new_n252), .out0(new_n255));
  aoai13aa1n04x5               g160(.a(new_n255), .b(new_n253), .c(new_n202), .d(new_n197), .o1(new_n256));
  xorb03aa1n02x5               g161(.a(new_n256), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  nor002aa1n10x5               g162(.a(\b[22] ), .b(\a[23] ), .o1(new_n258));
  nand42aa1n02x5               g163(.a(\b[22] ), .b(\a[23] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  norp02aa1n02x5               g165(.a(\b[23] ), .b(\a[24] ), .o1(new_n261));
  nand42aa1n02x5               g166(.a(\b[23] ), .b(\a[24] ), .o1(new_n262));
  norb02aa1n02x5               g167(.a(new_n262), .b(new_n261), .out0(new_n263));
  aoi112aa1n03x4               g168(.a(new_n258), .b(new_n263), .c(new_n256), .d(new_n260), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n258), .o1(new_n265));
  inv000aa1n02x5               g170(.a(new_n253), .o1(new_n266));
  inv000aa1n02x5               g171(.a(new_n255), .o1(new_n267));
  aoai13aa1n03x5               g172(.a(new_n260), .b(new_n267), .c(new_n198), .d(new_n266), .o1(new_n268));
  inv000aa1d42x5               g173(.a(new_n263), .o1(new_n269));
  aoi012aa1n03x5               g174(.a(new_n269), .b(new_n268), .c(new_n265), .o1(new_n270));
  nor002aa1n02x5               g175(.a(new_n270), .b(new_n264), .o1(\s[24] ));
  nona23aa1n12x5               g176(.a(new_n262), .b(new_n259), .c(new_n258), .d(new_n261), .out0(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  nano32aa1n03x7               g178(.a(new_n211), .b(new_n273), .c(new_n251), .d(new_n252), .out0(new_n274));
  inv000aa1n02x5               g179(.a(new_n274), .o1(new_n275));
  oaoi03aa1n02x5               g180(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .o1(new_n276));
  oab012aa1n04x5               g181(.a(new_n276), .b(new_n272), .c(new_n254), .out0(new_n277));
  nona32aa1n09x5               g182(.a(new_n235), .b(new_n272), .c(new_n244), .d(new_n240), .out0(new_n278));
  nand22aa1n06x5               g183(.a(new_n278), .b(new_n277), .o1(new_n279));
  inv000aa1n06x5               g184(.a(new_n279), .o1(new_n280));
  aoai13aa1n06x5               g185(.a(new_n280), .b(new_n275), .c(new_n202), .d(new_n197), .o1(new_n281));
  xorb03aa1n02x5               g186(.a(new_n281), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  nor042aa1n03x5               g187(.a(\b[24] ), .b(\a[25] ), .o1(new_n283));
  xorc02aa1n02x5               g188(.a(\a[25] ), .b(\b[24] ), .out0(new_n284));
  xorc02aa1n12x5               g189(.a(\a[26] ), .b(\b[25] ), .out0(new_n285));
  aoi112aa1n03x4               g190(.a(new_n283), .b(new_n285), .c(new_n281), .d(new_n284), .o1(new_n286));
  inv000aa1n02x5               g191(.a(new_n283), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n284), .b(new_n279), .c(new_n198), .d(new_n274), .o1(new_n288));
  inv000aa1d42x5               g193(.a(new_n285), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n289), .b(new_n288), .c(new_n287), .o1(new_n290));
  nor002aa1n02x5               g195(.a(new_n290), .b(new_n286), .o1(\s[26] ));
  aoi013aa1n02x4               g196(.a(new_n194), .b(new_n184), .c(new_n180), .d(new_n179), .o1(new_n292));
  oai012aa1n02x5               g197(.a(new_n292), .b(new_n158), .c(new_n200), .o1(new_n293));
  and002aa1n06x5               g198(.a(new_n285), .b(new_n284), .o(new_n294));
  nano22aa1d15x5               g199(.a(new_n253), .b(new_n294), .c(new_n273), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n293), .c(new_n130), .d(new_n201), .o1(new_n296));
  oao003aa1n02x5               g201(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n297));
  inv000aa1d42x5               g202(.a(new_n297), .o1(new_n298));
  aoi012aa1n12x5               g203(.a(new_n298), .b(new_n279), .c(new_n294), .o1(new_n299));
  nor042aa1n04x5               g204(.a(\b[26] ), .b(\a[27] ), .o1(new_n300));
  nanp02aa1n02x5               g205(.a(\b[26] ), .b(\a[27] ), .o1(new_n301));
  norb02aa1n02x5               g206(.a(new_n301), .b(new_n300), .out0(new_n302));
  xnbna2aa1n03x5               g207(.a(new_n302), .b(new_n299), .c(new_n296), .out0(\s[27] ));
  xorc02aa1n02x5               g208(.a(\a[28] ), .b(\b[27] ), .out0(new_n304));
  inv000aa1d42x5               g209(.a(new_n294), .o1(new_n305));
  aoai13aa1n06x5               g210(.a(new_n297), .b(new_n305), .c(new_n278), .d(new_n277), .o1(new_n306));
  aoi112aa1n03x4               g211(.a(new_n306), .b(new_n300), .c(new_n198), .d(new_n295), .o1(new_n307));
  nano22aa1n03x5               g212(.a(new_n307), .b(new_n301), .c(new_n304), .out0(new_n308));
  inv000aa1d42x5               g213(.a(\a[28] ), .o1(new_n309));
  inv000aa1d42x5               g214(.a(\b[27] ), .o1(new_n310));
  nanp02aa1n02x5               g215(.a(new_n310), .b(new_n309), .o1(new_n311));
  nanp02aa1n02x5               g216(.a(\b[27] ), .b(\a[28] ), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n300), .o1(new_n313));
  nanp03aa1n03x5               g218(.a(new_n299), .b(new_n296), .c(new_n313), .o1(new_n314));
  aoi022aa1n03x5               g219(.a(new_n314), .b(new_n301), .c(new_n311), .d(new_n312), .o1(new_n315));
  nor002aa1n02x5               g220(.a(new_n315), .b(new_n308), .o1(\s[28] ));
  nano32aa1n02x4               g221(.a(new_n300), .b(new_n312), .c(new_n301), .d(new_n311), .out0(new_n317));
  aoai13aa1n03x5               g222(.a(new_n317), .b(new_n306), .c(new_n198), .d(new_n295), .o1(new_n318));
  oaoi03aa1n02x5               g223(.a(new_n309), .b(new_n310), .c(new_n300), .o1(new_n319));
  xnrc02aa1n02x5               g224(.a(\b[28] ), .b(\a[29] ), .out0(new_n320));
  aoi012aa1n03x5               g225(.a(new_n320), .b(new_n318), .c(new_n319), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n317), .o1(new_n322));
  aoi012aa1n02x7               g227(.a(new_n322), .b(new_n299), .c(new_n296), .o1(new_n323));
  nano22aa1n03x5               g228(.a(new_n323), .b(new_n319), .c(new_n320), .out0(new_n324));
  norp02aa1n03x5               g229(.a(new_n321), .b(new_n324), .o1(\s[29] ));
  xorb03aa1n02x5               g230(.a(new_n101), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano32aa1d12x5               g231(.a(new_n320), .b(new_n302), .c(new_n311), .d(new_n312), .out0(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n306), .c(new_n198), .d(new_n295), .o1(new_n328));
  oao003aa1n02x5               g233(.a(\a[29] ), .b(\b[28] ), .c(new_n319), .carry(new_n329));
  xnrc02aa1n02x5               g234(.a(\b[29] ), .b(\a[30] ), .out0(new_n330));
  aoi012aa1n02x7               g235(.a(new_n330), .b(new_n328), .c(new_n329), .o1(new_n331));
  inv000aa1n02x5               g236(.a(new_n327), .o1(new_n332));
  aoi012aa1n02x7               g237(.a(new_n332), .b(new_n299), .c(new_n296), .o1(new_n333));
  nano22aa1n03x5               g238(.a(new_n333), .b(new_n329), .c(new_n330), .out0(new_n334));
  norp02aa1n03x5               g239(.a(new_n331), .b(new_n334), .o1(\s[30] ));
  nano23aa1n06x5               g240(.a(new_n330), .b(new_n320), .c(new_n304), .d(new_n302), .out0(new_n336));
  inv000aa1d42x5               g241(.a(new_n336), .o1(new_n337));
  aoi012aa1n02x7               g242(.a(new_n337), .b(new_n299), .c(new_n296), .o1(new_n338));
  oao003aa1n02x5               g243(.a(\a[30] ), .b(\b[29] ), .c(new_n329), .carry(new_n339));
  xnrc02aa1n02x5               g244(.a(\b[30] ), .b(\a[31] ), .out0(new_n340));
  nano22aa1n03x5               g245(.a(new_n338), .b(new_n339), .c(new_n340), .out0(new_n341));
  aoai13aa1n06x5               g246(.a(new_n336), .b(new_n306), .c(new_n198), .d(new_n295), .o1(new_n342));
  tech160nm_fiaoi012aa1n02p5x5 g247(.a(new_n340), .b(new_n342), .c(new_n339), .o1(new_n343));
  norp02aa1n03x5               g248(.a(new_n343), .b(new_n341), .o1(\s[31] ));
  xnbna2aa1n03x5               g249(.a(new_n142), .b(new_n108), .c(new_n113), .out0(\s[3] ));
  oaoi03aa1n02x5               g250(.a(\a[3] ), .b(\b[2] ), .c(new_n142), .o1(new_n346));
  xorb03aa1n02x5               g251(.a(new_n346), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g252(.a(new_n165), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g253(.a(\b[4] ), .b(\a[5] ), .o1(new_n349));
  oai013aa1n02x4               g254(.a(new_n349), .b(new_n145), .c(new_n114), .d(new_n125), .o1(new_n350));
  xorb03aa1n02x5               g255(.a(new_n350), .b(\b[5] ), .c(new_n124), .out0(\s[6] ));
  oaoi03aa1n03x5               g256(.a(\a[6] ), .b(\b[5] ), .c(new_n350), .o1(new_n352));
  xorb03aa1n02x5               g257(.a(new_n352), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g258(.a(new_n118), .b(new_n352), .c(new_n119), .o1(new_n354));
  xnrb03aa1n03x5               g259(.a(new_n354), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g260(.a(new_n130), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


