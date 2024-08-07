// Benchmark "adder" written by ABC on Thu Jul 18 03:31:37 2024

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
    new_n140, new_n141, new_n142, new_n144, new_n145, new_n146, new_n147,
    new_n148, new_n149, new_n150, new_n151, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n167, new_n168, new_n169, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n185, new_n186,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n203, new_n204, new_n205, new_n207, new_n208, new_n209, new_n210,
    new_n211, new_n212, new_n213, new_n216, new_n217, new_n218, new_n219,
    new_n220, new_n221, new_n222, new_n223, new_n224, new_n225, new_n226,
    new_n228, new_n229, new_n230, new_n231, new_n232, new_n233, new_n234,
    new_n235, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n245, new_n246, new_n247, new_n248, new_n249,
    new_n250, new_n251, new_n253, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n285, new_n286, new_n287, new_n288,
    new_n289, new_n290, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n299, new_n300, new_n301, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n311, new_n312,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n320, new_n321,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n329, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n339, new_n340, new_n341, new_n342, new_n344,
    new_n345, new_n347, new_n348, new_n349, new_n350, new_n351, new_n353,
    new_n354, new_n355, new_n356, new_n357, new_n359, new_n361, new_n362,
    new_n363, new_n364, new_n366, new_n367, new_n368, new_n370;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n03x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\a[2] ), .o1(new_n99));
  inv000aa1d42x5               g004(.a(\b[1] ), .o1(new_n100));
  nand42aa1n03x5               g005(.a(new_n100), .b(new_n99), .o1(new_n101));
  nand22aa1n09x5               g006(.a(\b[0] ), .b(\a[1] ), .o1(new_n102));
  aoi012aa1n12x5               g007(.a(new_n102), .b(\a[2] ), .c(\b[1] ), .o1(new_n103));
  tech160nm_finand02aa1n03p5x5 g008(.a(\b[1] ), .b(\a[2] ), .o1(new_n104));
  nand42aa1n06x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nor022aa1n16x5               g010(.a(\b[2] ), .b(\a[3] ), .o1(new_n106));
  nanb03aa1n06x5               g011(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n107));
  aoi012aa1n06x5               g012(.a(new_n107), .b(new_n101), .c(new_n103), .o1(new_n108));
  oaih22aa1n06x5               g013(.a(\a[3] ), .b(\b[2] ), .c(\b[3] ), .d(\a[4] ), .o1(new_n109));
  nanp02aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[4] ), .b(\a[5] ), .o1(new_n111));
  aoi012aa1n02x7               g016(.a(new_n111), .b(\a[6] ), .c(\b[5] ), .o1(new_n112));
  nor022aa1n06x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi012aa1n02x5               g018(.a(new_n113), .b(\a[4] ), .c(\b[3] ), .o1(new_n114));
  nand02aa1n08x5               g019(.a(\b[7] ), .b(\a[8] ), .o1(new_n115));
  nor002aa1n12x5               g020(.a(\b[6] ), .b(\a[7] ), .o1(new_n116));
  nand22aa1n03x5               g021(.a(\b[6] ), .b(\a[7] ), .o1(new_n117));
  nor002aa1n06x5               g022(.a(\b[5] ), .b(\a[6] ), .o1(new_n118));
  nona23aa1n03x5               g023(.a(new_n117), .b(new_n115), .c(new_n118), .d(new_n116), .out0(new_n119));
  nano32aa1n03x7               g024(.a(new_n119), .b(new_n114), .c(new_n112), .d(new_n110), .out0(new_n120));
  oai012aa1n12x5               g025(.a(new_n120), .b(new_n108), .c(new_n109), .o1(new_n121));
  oa0012aa1n03x5               g026(.a(new_n115), .b(new_n116), .c(new_n113), .o(new_n122));
  oai022aa1n04x7               g027(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n123));
  nand42aa1n02x5               g028(.a(\b[5] ), .b(\a[6] ), .o1(new_n124));
  nanp03aa1n03x5               g029(.a(new_n115), .b(new_n124), .c(new_n117), .o1(new_n125));
  nor002aa1n03x5               g030(.a(new_n118), .b(new_n111), .o1(new_n126));
  nor043aa1n02x5               g031(.a(new_n125), .b(new_n126), .c(new_n123), .o1(new_n127));
  norp02aa1n06x5               g032(.a(new_n127), .b(new_n122), .o1(new_n128));
  nand02aa1d06x5               g033(.a(new_n121), .b(new_n128), .o1(new_n129));
  xnrc02aa1n12x5               g034(.a(\b[8] ), .b(\a[9] ), .out0(new_n130));
  inv000aa1d42x5               g035(.a(new_n130), .o1(new_n131));
  nanp02aa1n02x5               g036(.a(new_n129), .b(new_n131), .o1(new_n132));
  xnrc02aa1n12x5               g037(.a(\b[9] ), .b(\a[10] ), .out0(new_n133));
  inv000aa1d42x5               g038(.a(new_n133), .o1(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n132), .c(new_n98), .out0(\s[10] ));
  aoai13aa1n06x5               g040(.a(new_n134), .b(new_n97), .c(new_n129), .d(new_n131), .o1(new_n136));
  nanp02aa1n04x5               g041(.a(\b[9] ), .b(\a[10] ), .o1(new_n137));
  oai022aa1d18x5               g042(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n138));
  nanp02aa1n02x5               g043(.a(new_n138), .b(new_n137), .o1(new_n139));
  nor002aa1n20x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nanp02aa1n12x5               g045(.a(\b[10] ), .b(\a[11] ), .o1(new_n141));
  norb02aa1n09x5               g046(.a(new_n141), .b(new_n140), .out0(new_n142));
  xnbna2aa1n03x5               g047(.a(new_n142), .b(new_n136), .c(new_n139), .out0(\s[11] ));
  aob012aa1n02x5               g048(.a(new_n142), .b(new_n136), .c(new_n139), .out0(new_n144));
  inv000aa1n02x5               g049(.a(new_n140), .o1(new_n145));
  inv000aa1d42x5               g050(.a(new_n142), .o1(new_n146));
  aoai13aa1n02x5               g051(.a(new_n145), .b(new_n146), .c(new_n136), .d(new_n139), .o1(new_n147));
  nor002aa1n06x5               g052(.a(\b[11] ), .b(\a[12] ), .o1(new_n148));
  nand02aa1d16x5               g053(.a(\b[11] ), .b(\a[12] ), .o1(new_n149));
  nanb02aa1n02x5               g054(.a(new_n148), .b(new_n149), .out0(new_n150));
  aoib12aa1n02x5               g055(.a(new_n140), .b(new_n149), .c(new_n148), .out0(new_n151));
  aboi22aa1n03x5               g056(.a(new_n150), .b(new_n147), .c(new_n144), .d(new_n151), .out0(\s[12] ));
  nona23aa1n09x5               g057(.a(new_n149), .b(new_n141), .c(new_n140), .d(new_n148), .out0(new_n153));
  nor043aa1d12x5               g058(.a(new_n153), .b(new_n133), .c(new_n130), .o1(new_n154));
  nano22aa1n03x7               g059(.a(new_n148), .b(new_n141), .c(new_n149), .out0(new_n155));
  oai112aa1n06x5               g060(.a(new_n138), .b(new_n137), .c(\b[10] ), .d(\a[11] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n148), .b(new_n140), .c(new_n149), .o1(new_n157));
  oaib12aa1n06x5               g062(.a(new_n157), .b(new_n156), .c(new_n155), .out0(new_n158));
  xnrc02aa1n12x5               g063(.a(\b[12] ), .b(\a[13] ), .out0(new_n159));
  inv000aa1d42x5               g064(.a(new_n159), .o1(new_n160));
  aoai13aa1n03x5               g065(.a(new_n160), .b(new_n158), .c(new_n129), .d(new_n154), .o1(new_n161));
  inv030aa1n02x5               g066(.a(new_n156), .o1(new_n162));
  oaoi03aa1n02x5               g067(.a(\a[12] ), .b(\b[11] ), .c(new_n145), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n163), .b(new_n160), .c(new_n162), .d(new_n155), .o1(new_n164));
  aobi12aa1n02x5               g069(.a(new_n164), .b(new_n129), .c(new_n154), .out0(new_n165));
  norb02aa1n02x5               g070(.a(new_n161), .b(new_n165), .out0(\s[13] ));
  nor042aa1n03x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  inv040aa1n03x5               g072(.a(new_n167), .o1(new_n168));
  tech160nm_fixnrc02aa1n04x5   g073(.a(\b[13] ), .b(\a[14] ), .out0(new_n169));
  xobna2aa1n03x5               g074(.a(new_n169), .b(new_n161), .c(new_n168), .out0(\s[14] ));
  nor042aa1n06x5               g075(.a(new_n169), .b(new_n159), .o1(new_n171));
  aoai13aa1n06x5               g076(.a(new_n171), .b(new_n158), .c(new_n129), .d(new_n154), .o1(new_n172));
  oaoi03aa1n03x5               g077(.a(\a[14] ), .b(\b[13] ), .c(new_n168), .o1(new_n173));
  inv000aa1n02x5               g078(.a(new_n173), .o1(new_n174));
  nor002aa1d32x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n03x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  xnbna2aa1n03x5               g082(.a(new_n177), .b(new_n172), .c(new_n174), .out0(\s[15] ));
  aob012aa1n03x5               g083(.a(new_n177), .b(new_n172), .c(new_n174), .out0(new_n179));
  inv000aa1d42x5               g084(.a(new_n175), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n177), .o1(new_n181));
  aoai13aa1n02x5               g086(.a(new_n180), .b(new_n181), .c(new_n172), .d(new_n174), .o1(new_n182));
  nor002aa1n16x5               g087(.a(\b[15] ), .b(\a[16] ), .o1(new_n183));
  nand02aa1n06x5               g088(.a(\b[15] ), .b(\a[16] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  aoib12aa1n02x5               g090(.a(new_n175), .b(new_n184), .c(new_n183), .out0(new_n186));
  aoi022aa1n02x7               g091(.a(new_n182), .b(new_n185), .c(new_n179), .d(new_n186), .o1(\s[16] ));
  nona23aa1d18x5               g092(.a(new_n184), .b(new_n176), .c(new_n175), .d(new_n183), .out0(new_n188));
  aoai13aa1n06x5               g093(.a(new_n171), .b(new_n163), .c(new_n162), .d(new_n155), .o1(new_n189));
  aoi012aa1n12x5               g094(.a(new_n188), .b(new_n189), .c(new_n174), .o1(new_n190));
  nona32aa1n09x5               g095(.a(new_n154), .b(new_n188), .c(new_n169), .d(new_n159), .out0(new_n191));
  aoi012aa1n06x5               g096(.a(new_n183), .b(new_n175), .c(new_n184), .o1(new_n192));
  aoai13aa1n12x5               g097(.a(new_n192), .b(new_n191), .c(new_n121), .d(new_n128), .o1(new_n193));
  xorc02aa1n12x5               g098(.a(\a[17] ), .b(\b[16] ), .out0(new_n194));
  oai012aa1n06x5               g099(.a(new_n194), .b(new_n193), .c(new_n190), .o1(new_n195));
  inv000aa1n02x5               g100(.a(new_n188), .o1(new_n196));
  aoai13aa1n12x5               g101(.a(new_n196), .b(new_n173), .c(new_n158), .d(new_n171), .o1(new_n197));
  nona23aa1n03x5               g102(.a(new_n134), .b(new_n142), .c(new_n130), .d(new_n150), .out0(new_n198));
  nano22aa1n03x7               g103(.a(new_n198), .b(new_n171), .c(new_n196), .out0(new_n199));
  nand02aa1d06x5               g104(.a(new_n129), .b(new_n199), .o1(new_n200));
  nano32aa1n02x4               g105(.a(new_n194), .b(new_n200), .c(new_n197), .d(new_n192), .out0(new_n201));
  norb02aa1n02x5               g106(.a(new_n195), .b(new_n201), .out0(\s[17] ));
  inv000aa1d42x5               g107(.a(\a[17] ), .o1(new_n203));
  nanb02aa1n02x5               g108(.a(\b[16] ), .b(new_n203), .out0(new_n204));
  xorc02aa1n12x5               g109(.a(\a[18] ), .b(\b[17] ), .out0(new_n205));
  xnbna2aa1n03x5               g110(.a(new_n205), .b(new_n195), .c(new_n204), .out0(\s[18] ));
  and002aa1n02x5               g111(.a(new_n205), .b(new_n194), .o(new_n207));
  oai012aa1n06x5               g112(.a(new_n207), .b(new_n193), .c(new_n190), .o1(new_n208));
  oaoi03aa1n02x5               g113(.a(\a[18] ), .b(\b[17] ), .c(new_n204), .o1(new_n209));
  inv000aa1d42x5               g114(.a(new_n209), .o1(new_n210));
  nor002aa1d32x5               g115(.a(\b[18] ), .b(\a[19] ), .o1(new_n211));
  nand02aa1n06x5               g116(.a(\b[18] ), .b(\a[19] ), .o1(new_n212));
  norb02aa1n03x5               g117(.a(new_n212), .b(new_n211), .out0(new_n213));
  xnbna2aa1n03x5               g118(.a(new_n213), .b(new_n208), .c(new_n210), .out0(\s[19] ));
  xnrc02aa1n02x5               g119(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nanp03aa1d12x5               g120(.a(new_n200), .b(new_n197), .c(new_n192), .o1(new_n216));
  aoai13aa1n06x5               g121(.a(new_n213), .b(new_n209), .c(new_n216), .d(new_n207), .o1(new_n217));
  inv000aa1d42x5               g122(.a(new_n211), .o1(new_n218));
  inv040aa1n03x5               g123(.a(new_n213), .o1(new_n219));
  aoai13aa1n03x5               g124(.a(new_n218), .b(new_n219), .c(new_n208), .d(new_n210), .o1(new_n220));
  nor002aa1d32x5               g125(.a(\b[19] ), .b(\a[20] ), .o1(new_n221));
  nand42aa1n16x5               g126(.a(\b[19] ), .b(\a[20] ), .o1(new_n222));
  norb02aa1n02x5               g127(.a(new_n222), .b(new_n221), .out0(new_n223));
  inv000aa1d42x5               g128(.a(\a[19] ), .o1(new_n224));
  inv000aa1d42x5               g129(.a(\b[18] ), .o1(new_n225));
  aboi22aa1n03x5               g130(.a(new_n221), .b(new_n222), .c(new_n224), .d(new_n225), .out0(new_n226));
  aoi022aa1n03x5               g131(.a(new_n220), .b(new_n223), .c(new_n217), .d(new_n226), .o1(\s[20] ));
  nano32aa1n03x7               g132(.a(new_n219), .b(new_n205), .c(new_n194), .d(new_n223), .out0(new_n228));
  tech160nm_fioai012aa1n05x5   g133(.a(new_n228), .b(new_n193), .c(new_n190), .o1(new_n229));
  nanb03aa1n09x5               g134(.a(new_n221), .b(new_n222), .c(new_n212), .out0(new_n230));
  nand02aa1d08x5               g135(.a(\b[17] ), .b(\a[18] ), .o1(new_n231));
  oai022aa1d24x5               g136(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n232));
  oai112aa1n06x5               g137(.a(new_n232), .b(new_n231), .c(\b[18] ), .d(\a[19] ), .o1(new_n233));
  aoi012aa1n12x5               g138(.a(new_n221), .b(new_n211), .c(new_n222), .o1(new_n234));
  oai012aa1d24x5               g139(.a(new_n234), .b(new_n233), .c(new_n230), .o1(new_n235));
  xnrc02aa1n12x5               g140(.a(\b[20] ), .b(\a[21] ), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n236), .o1(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n235), .c(new_n216), .d(new_n228), .o1(new_n238));
  nano22aa1n03x7               g143(.a(new_n221), .b(new_n212), .c(new_n222), .out0(new_n239));
  oai012aa1n02x5               g144(.a(new_n231), .b(\b[18] ), .c(\a[19] ), .o1(new_n240));
  norb02aa1n02x5               g145(.a(new_n232), .b(new_n240), .out0(new_n241));
  inv000aa1n02x5               g146(.a(new_n234), .o1(new_n242));
  aoi112aa1n02x5               g147(.a(new_n242), .b(new_n237), .c(new_n241), .d(new_n239), .o1(new_n243));
  aobi12aa1n03x7               g148(.a(new_n238), .b(new_n243), .c(new_n229), .out0(\s[21] ));
  inv000aa1d42x5               g149(.a(new_n235), .o1(new_n245));
  nor042aa1n03x5               g150(.a(\b[20] ), .b(\a[21] ), .o1(new_n246));
  inv000aa1n03x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n03x5               g152(.a(new_n247), .b(new_n236), .c(new_n229), .d(new_n245), .o1(new_n248));
  xnrc02aa1n12x5               g153(.a(\b[21] ), .b(\a[22] ), .out0(new_n249));
  inv000aa1d42x5               g154(.a(new_n249), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n249), .b(new_n246), .out0(new_n251));
  aoi022aa1n03x5               g156(.a(new_n248), .b(new_n250), .c(new_n238), .d(new_n251), .o1(\s[22] ));
  nor042aa1n06x5               g157(.a(new_n249), .b(new_n236), .o1(new_n253));
  and002aa1n02x5               g158(.a(new_n228), .b(new_n253), .o(new_n254));
  tech160nm_fioai012aa1n05x5   g159(.a(new_n254), .b(new_n193), .c(new_n190), .o1(new_n255));
  oao003aa1n02x5               g160(.a(\a[22] ), .b(\b[21] ), .c(new_n247), .carry(new_n256));
  inv030aa1n02x5               g161(.a(new_n256), .o1(new_n257));
  tech160nm_fiaoi012aa1n03p5x5 g162(.a(new_n257), .b(new_n235), .c(new_n253), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  xorc02aa1n12x5               g164(.a(\a[23] ), .b(\b[22] ), .out0(new_n260));
  aoai13aa1n06x5               g165(.a(new_n260), .b(new_n259), .c(new_n216), .d(new_n254), .o1(new_n261));
  aoi112aa1n02x5               g166(.a(new_n260), .b(new_n257), .c(new_n235), .d(new_n253), .o1(new_n262));
  aobi12aa1n02x7               g167(.a(new_n261), .b(new_n262), .c(new_n255), .out0(\s[23] ));
  nor042aa1n09x5               g168(.a(\b[22] ), .b(\a[23] ), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n264), .o1(new_n265));
  inv000aa1d42x5               g170(.a(new_n260), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n265), .b(new_n266), .c(new_n255), .d(new_n258), .o1(new_n267));
  xorc02aa1n03x5               g172(.a(\a[24] ), .b(\b[23] ), .out0(new_n268));
  norp02aa1n02x5               g173(.a(new_n268), .b(new_n264), .o1(new_n269));
  aoi022aa1n02x7               g174(.a(new_n267), .b(new_n268), .c(new_n261), .d(new_n269), .o1(\s[24] ));
  inv000aa1n02x5               g175(.a(new_n228), .o1(new_n271));
  and002aa1n06x5               g176(.a(new_n268), .b(new_n260), .o(new_n272));
  nano22aa1n03x7               g177(.a(new_n271), .b(new_n272), .c(new_n253), .out0(new_n273));
  oai012aa1n04x7               g178(.a(new_n273), .b(new_n193), .c(new_n190), .o1(new_n274));
  aoai13aa1n04x5               g179(.a(new_n253), .b(new_n242), .c(new_n241), .d(new_n239), .o1(new_n275));
  inv030aa1n02x5               g180(.a(new_n272), .o1(new_n276));
  oao003aa1n02x5               g181(.a(\a[24] ), .b(\b[23] ), .c(new_n265), .carry(new_n277));
  aoai13aa1n06x5               g182(.a(new_n277), .b(new_n276), .c(new_n275), .d(new_n256), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  aoai13aa1n06x5               g184(.a(new_n279), .b(new_n278), .c(new_n216), .d(new_n273), .o1(new_n280));
  aoai13aa1n09x5               g185(.a(new_n272), .b(new_n257), .c(new_n235), .d(new_n253), .o1(new_n281));
  inv000aa1d42x5               g186(.a(new_n279), .o1(new_n282));
  and003aa1n02x5               g187(.a(new_n281), .b(new_n282), .c(new_n277), .o(new_n283));
  aobi12aa1n02x7               g188(.a(new_n280), .b(new_n283), .c(new_n274), .out0(\s[25] ));
  inv000aa1n02x5               g189(.a(new_n278), .o1(new_n285));
  nor042aa1n03x5               g190(.a(\b[24] ), .b(\a[25] ), .o1(new_n286));
  inv040aa1n03x5               g191(.a(new_n286), .o1(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n282), .c(new_n274), .d(new_n285), .o1(new_n288));
  tech160nm_fixorc02aa1n03p5x5 g193(.a(\a[26] ), .b(\b[25] ), .out0(new_n289));
  norp02aa1n02x5               g194(.a(new_n289), .b(new_n286), .o1(new_n290));
  aoi022aa1n02x7               g195(.a(new_n288), .b(new_n289), .c(new_n280), .d(new_n290), .o1(\s[26] ));
  and002aa1n12x5               g196(.a(new_n289), .b(new_n279), .o(new_n292));
  nano32aa1d15x5               g197(.a(new_n271), .b(new_n292), .c(new_n253), .d(new_n272), .out0(new_n293));
  oai012aa1n12x5               g198(.a(new_n293), .b(new_n193), .c(new_n190), .o1(new_n294));
  inv000aa1d42x5               g199(.a(new_n292), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[26] ), .b(\b[25] ), .c(new_n287), .carry(new_n296));
  aoai13aa1n12x5               g201(.a(new_n296), .b(new_n295), .c(new_n281), .d(new_n277), .o1(new_n297));
  xorc02aa1n12x5               g202(.a(\a[27] ), .b(\b[26] ), .out0(new_n298));
  aoai13aa1n06x5               g203(.a(new_n298), .b(new_n297), .c(new_n216), .d(new_n293), .o1(new_n299));
  inv000aa1n02x5               g204(.a(new_n296), .o1(new_n300));
  aoi112aa1n02x5               g205(.a(new_n298), .b(new_n300), .c(new_n278), .d(new_n292), .o1(new_n301));
  aobi12aa1n02x7               g206(.a(new_n299), .b(new_n301), .c(new_n294), .out0(\s[27] ));
  tech160nm_fiaoi012aa1n05x5   g207(.a(new_n300), .b(new_n278), .c(new_n292), .o1(new_n303));
  norp02aa1n02x5               g208(.a(\b[26] ), .b(\a[27] ), .o1(new_n304));
  inv000aa1n03x5               g209(.a(new_n304), .o1(new_n305));
  inv030aa1n02x5               g210(.a(new_n298), .o1(new_n306));
  aoai13aa1n03x5               g211(.a(new_n305), .b(new_n306), .c(new_n294), .d(new_n303), .o1(new_n307));
  tech160nm_fixorc02aa1n03p5x5 g212(.a(\a[28] ), .b(\b[27] ), .out0(new_n308));
  norp02aa1n02x5               g213(.a(new_n308), .b(new_n304), .o1(new_n309));
  aoi022aa1n03x5               g214(.a(new_n307), .b(new_n308), .c(new_n299), .d(new_n309), .o1(\s[28] ));
  and002aa1n02x5               g215(.a(new_n308), .b(new_n298), .o(new_n311));
  aoai13aa1n06x5               g216(.a(new_n311), .b(new_n297), .c(new_n216), .d(new_n293), .o1(new_n312));
  inv000aa1d42x5               g217(.a(new_n311), .o1(new_n313));
  oao003aa1n02x5               g218(.a(\a[28] ), .b(\b[27] ), .c(new_n305), .carry(new_n314));
  aoai13aa1n02x7               g219(.a(new_n314), .b(new_n313), .c(new_n294), .d(new_n303), .o1(new_n315));
  tech160nm_fixorc02aa1n03p5x5 g220(.a(\a[29] ), .b(\b[28] ), .out0(new_n316));
  norb02aa1n02x5               g221(.a(new_n314), .b(new_n316), .out0(new_n317));
  aoi022aa1n03x5               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g223(.a(new_n102), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x5               g224(.a(new_n306), .b(new_n308), .c(new_n316), .out0(new_n320));
  aoai13aa1n03x5               g225(.a(new_n320), .b(new_n297), .c(new_n216), .d(new_n293), .o1(new_n321));
  inv000aa1n02x5               g226(.a(new_n320), .o1(new_n322));
  inv000aa1d42x5               g227(.a(\b[28] ), .o1(new_n323));
  inv000aa1d42x5               g228(.a(\a[29] ), .o1(new_n324));
  oaib12aa1n02x5               g229(.a(new_n314), .b(\b[28] ), .c(new_n324), .out0(new_n325));
  oaib12aa1n02x5               g230(.a(new_n325), .b(new_n323), .c(\a[29] ), .out0(new_n326));
  aoai13aa1n02x7               g231(.a(new_n326), .b(new_n322), .c(new_n294), .d(new_n303), .o1(new_n327));
  xorc02aa1n02x5               g232(.a(\a[30] ), .b(\b[29] ), .out0(new_n328));
  oaoi13aa1n02x5               g233(.a(new_n328), .b(new_n325), .c(new_n324), .d(new_n323), .o1(new_n329));
  aoi022aa1n03x5               g234(.a(new_n327), .b(new_n328), .c(new_n321), .d(new_n329), .o1(\s[30] ));
  nanb02aa1n02x5               g235(.a(\b[30] ), .b(\a[31] ), .out0(new_n331));
  nanb02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  nanp02aa1n02x5               g237(.a(new_n332), .b(new_n331), .o1(new_n333));
  nano32aa1n02x5               g238(.a(new_n306), .b(new_n328), .c(new_n308), .d(new_n316), .out0(new_n334));
  aoai13aa1n03x5               g239(.a(new_n334), .b(new_n297), .c(new_n216), .d(new_n293), .o1(new_n335));
  inv000aa1n02x5               g240(.a(new_n334), .o1(new_n336));
  norp02aa1n02x5               g241(.a(\b[29] ), .b(\a[30] ), .o1(new_n337));
  aoi022aa1n02x5               g242(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n338));
  aoi012aa1n02x5               g243(.a(new_n337), .b(new_n325), .c(new_n338), .o1(new_n339));
  aoai13aa1n02x7               g244(.a(new_n339), .b(new_n336), .c(new_n294), .d(new_n303), .o1(new_n340));
  oai112aa1n02x5               g245(.a(new_n331), .b(new_n332), .c(\b[29] ), .d(\a[30] ), .o1(new_n341));
  aoi012aa1n02x5               g246(.a(new_n341), .b(new_n325), .c(new_n338), .o1(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n333), .c(new_n335), .d(new_n342), .o1(\s[31] ));
  nanb02aa1n02x5               g248(.a(new_n106), .b(new_n105), .out0(new_n344));
  oaoi03aa1n02x5               g249(.a(new_n99), .b(new_n100), .c(new_n102), .o1(new_n345));
  aoi012aa1n02x5               g250(.a(new_n108), .b(new_n344), .c(new_n345), .o1(\s[3] ));
  nano22aa1n02x4               g251(.a(new_n106), .b(new_n104), .c(new_n105), .out0(new_n347));
  aob012aa1n02x5               g252(.a(new_n347), .b(new_n103), .c(new_n101), .out0(new_n348));
  nanb02aa1n02x5               g253(.a(new_n109), .b(new_n348), .out0(new_n349));
  xorc02aa1n02x5               g254(.a(\a[4] ), .b(\b[3] ), .out0(new_n350));
  norp02aa1n02x5               g255(.a(new_n350), .b(new_n106), .o1(new_n351));
  aoi022aa1n02x5               g256(.a(new_n349), .b(new_n350), .c(new_n348), .d(new_n351), .o1(\s[4] ));
  nanp02aa1n02x5               g257(.a(\b[3] ), .b(\a[4] ), .o1(new_n353));
  nano22aa1n02x4               g258(.a(new_n111), .b(new_n110), .c(new_n353), .out0(new_n354));
  oai012aa1n02x5               g259(.a(new_n354), .b(new_n108), .c(new_n109), .o1(new_n355));
  inv000aa1d42x5               g260(.a(new_n111), .o1(new_n356));
  aoi022aa1n02x5               g261(.a(new_n349), .b(new_n353), .c(new_n110), .d(new_n356), .o1(new_n357));
  norb02aa1n02x5               g262(.a(new_n355), .b(new_n357), .out0(\s[5] ));
  norb02aa1n02x5               g263(.a(new_n124), .b(new_n118), .out0(new_n359));
  xnbna2aa1n03x5               g264(.a(new_n359), .b(new_n355), .c(new_n356), .out0(\s[6] ));
  nanp02aa1n02x5               g265(.a(new_n355), .b(new_n356), .o1(new_n361));
  norb02aa1n02x5               g266(.a(new_n117), .b(new_n116), .out0(new_n362));
  aoai13aa1n02x5               g267(.a(new_n362), .b(new_n118), .c(new_n361), .d(new_n359), .o1(new_n363));
  aoi112aa1n02x5               g268(.a(new_n362), .b(new_n118), .c(new_n361), .d(new_n359), .o1(new_n364));
  norb02aa1n02x5               g269(.a(new_n363), .b(new_n364), .out0(\s[7] ));
  oai012aa1n02x5               g270(.a(new_n363), .b(\b[6] ), .c(\a[7] ), .o1(new_n366));
  norb02aa1n02x5               g271(.a(new_n115), .b(new_n113), .out0(new_n367));
  aoib12aa1n02x5               g272(.a(new_n116), .b(new_n115), .c(new_n113), .out0(new_n368));
  aoi022aa1n02x5               g273(.a(new_n366), .b(new_n367), .c(new_n363), .d(new_n368), .o1(\s[8] ));
  norp03aa1n02x5               g274(.a(new_n127), .b(new_n131), .c(new_n122), .o1(new_n370));
  aoi022aa1n02x5               g275(.a(new_n129), .b(new_n131), .c(new_n121), .d(new_n370), .o1(\s[9] ));
endmodule


