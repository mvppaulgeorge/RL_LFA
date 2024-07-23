// Benchmark "adder" written by ABC on Wed Jul 17 20:35:53 2024

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
    new_n125, new_n126, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n180, new_n181, new_n182, new_n183, new_n184, new_n185,
    new_n186, new_n188, new_n189, new_n190, new_n191, new_n192, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n222, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n254, new_n255, new_n256,
    new_n257, new_n258, new_n259, new_n261, new_n262, new_n263, new_n264,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n272,
    new_n273, new_n274, new_n275, new_n276, new_n277, new_n279, new_n280,
    new_n281, new_n282, new_n283, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n292, new_n293, new_n294, new_n295,
    new_n296, new_n297, new_n298, new_n299, new_n300, new_n302, new_n303,
    new_n304, new_n305, new_n306, new_n307, new_n308, new_n309, new_n310,
    new_n312, new_n313, new_n314, new_n315, new_n316, new_n317, new_n318,
    new_n319, new_n320, new_n321, new_n323, new_n324, new_n325, new_n326,
    new_n327, new_n328, new_n329, new_n330, new_n333, new_n334, new_n335,
    new_n336, new_n337, new_n338, new_n339, new_n340, new_n341, new_n342,
    new_n344, new_n345, new_n346, new_n347, new_n348, new_n349, new_n350,
    new_n351, new_n352, new_n353, new_n354, new_n357, new_n358, new_n359,
    new_n361, new_n363, new_n364, new_n365, new_n367, new_n368, new_n369,
    new_n371, new_n372, new_n373, new_n374, new_n376;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  norp02aa1n04x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(new_n97), .o1(new_n98));
  oai112aa1n04x5               g003(.a(\a[1] ), .b(\b[0] ), .c(\b[1] ), .d(\a[2] ), .o1(new_n99));
  nand42aa1n20x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  nor002aa1n20x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand02aa1d24x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  norb02aa1n03x5               g007(.a(new_n102), .b(new_n101), .out0(new_n103));
  nand03aa1n04x5               g008(.a(new_n103), .b(new_n99), .c(new_n100), .o1(new_n104));
  oab012aa1n06x5               g009(.a(new_n101), .b(\a[4] ), .c(\b[3] ), .out0(new_n105));
  nor002aa1d32x5               g010(.a(\b[7] ), .b(\a[8] ), .o1(new_n106));
  aoi122aa1n12x5               g011(.a(new_n106), .b(\b[6] ), .c(\a[7] ), .d(\b[3] ), .e(\a[4] ), .o1(new_n107));
  nand42aa1d28x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  oai012aa1n18x5               g013(.a(new_n108), .b(\b[6] ), .c(\a[7] ), .o1(new_n109));
  oai022aa1n02x5               g014(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n110));
  aoi022aa1d24x5               g015(.a(\b[7] ), .b(\a[8] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n111));
  nona23aa1n09x5               g016(.a(new_n107), .b(new_n111), .c(new_n110), .d(new_n109), .out0(new_n112));
  inv040aa1d32x5               g017(.a(\a[7] ), .o1(new_n113));
  inv040aa1d32x5               g018(.a(\b[6] ), .o1(new_n114));
  nand42aa1n04x5               g019(.a(new_n114), .b(new_n113), .o1(new_n115));
  oaoi03aa1n12x5               g020(.a(\a[8] ), .b(\b[7] ), .c(new_n115), .o1(new_n116));
  and002aa1n12x5               g021(.a(\b[7] ), .b(\a[8] ), .o(new_n117));
  aoi112aa1n09x5               g022(.a(new_n117), .b(new_n106), .c(\a[7] ), .d(\b[6] ), .o1(new_n118));
  nor002aa1d24x5               g023(.a(\b[5] ), .b(\a[6] ), .o1(new_n119));
  nor002aa1d32x5               g024(.a(\b[4] ), .b(\a[5] ), .o1(new_n120));
  oab012aa1n09x5               g025(.a(new_n109), .b(new_n119), .c(new_n120), .out0(new_n121));
  aoi012aa1d24x5               g026(.a(new_n116), .b(new_n121), .c(new_n118), .o1(new_n122));
  aoai13aa1n12x5               g027(.a(new_n122), .b(new_n112), .c(new_n104), .d(new_n105), .o1(new_n123));
  xorc02aa1n12x5               g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  nanp02aa1n02x5               g029(.a(new_n123), .b(new_n124), .o1(new_n125));
  xorc02aa1n12x5               g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n98), .out0(\s[10] ));
  aoai13aa1n06x5               g032(.a(new_n126), .b(new_n97), .c(new_n123), .d(new_n124), .o1(new_n128));
  nanp02aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  oai022aa1d24x5               g034(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n129), .o1(new_n131));
  nor022aa1n12x5               g036(.a(\b[10] ), .b(\a[11] ), .o1(new_n132));
  nand42aa1d28x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  norb02aa1n02x5               g038(.a(new_n133), .b(new_n132), .out0(new_n134));
  xnbna2aa1n03x5               g039(.a(new_n134), .b(new_n128), .c(new_n131), .out0(\s[11] ));
  inv020aa1d32x5               g040(.a(\b[10] ), .o1(new_n136));
  nanb02aa1n12x5               g041(.a(\a[11] ), .b(new_n136), .out0(new_n137));
  aob012aa1n03x5               g042(.a(new_n134), .b(new_n128), .c(new_n131), .out0(new_n138));
  nor002aa1d32x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nand42aa1d28x5               g044(.a(\b[11] ), .b(\a[12] ), .o1(new_n140));
  norb02aa1n02x5               g045(.a(new_n140), .b(new_n139), .out0(new_n141));
  norb03aa1n02x5               g046(.a(new_n140), .b(new_n132), .c(new_n139), .out0(new_n142));
  nanp02aa1n03x5               g047(.a(new_n138), .b(new_n142), .o1(new_n143));
  aoai13aa1n03x5               g048(.a(new_n143), .b(new_n141), .c(new_n138), .d(new_n137), .o1(\s[12] ));
  nano23aa1n06x5               g049(.a(new_n132), .b(new_n139), .c(new_n140), .d(new_n133), .out0(new_n145));
  nand23aa1n06x5               g050(.a(new_n145), .b(new_n124), .c(new_n126), .o1(new_n146));
  inv000aa1d42x5               g051(.a(new_n146), .o1(new_n147));
  nanp02aa1n02x5               g052(.a(new_n123), .b(new_n147), .o1(new_n148));
  nand22aa1n04x5               g053(.a(\b[0] ), .b(\a[1] ), .o1(new_n149));
  oab012aa1n06x5               g054(.a(new_n149), .b(\a[2] ), .c(\b[1] ), .out0(new_n150));
  nano23aa1n03x7               g055(.a(new_n150), .b(new_n101), .c(new_n102), .d(new_n100), .out0(new_n151));
  inv000aa1n02x5               g056(.a(new_n105), .o1(new_n152));
  nanp02aa1n04x5               g057(.a(\b[3] ), .b(\a[4] ), .o1(new_n153));
  oai122aa1n02x7               g058(.a(new_n153), .b(new_n113), .c(new_n114), .d(\a[8] ), .e(\b[7] ), .o1(new_n154));
  nona22aa1n03x5               g059(.a(new_n111), .b(new_n120), .c(new_n119), .out0(new_n155));
  nor003aa1n03x5               g060(.a(new_n155), .b(new_n154), .c(new_n109), .o1(new_n156));
  oai012aa1n04x7               g061(.a(new_n156), .b(new_n151), .c(new_n152), .o1(new_n157));
  nanb03aa1n12x5               g062(.a(new_n139), .b(new_n140), .c(new_n133), .out0(new_n158));
  nanp03aa1d12x5               g063(.a(new_n130), .b(new_n137), .c(new_n129), .o1(new_n159));
  aoi012aa1d18x5               g064(.a(new_n139), .b(new_n132), .c(new_n140), .o1(new_n160));
  oai012aa1d24x5               g065(.a(new_n160), .b(new_n159), .c(new_n158), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n146), .c(new_n157), .d(new_n122), .o1(new_n163));
  nor002aa1d32x5               g068(.a(\b[12] ), .b(\a[13] ), .o1(new_n164));
  nand02aa1d16x5               g069(.a(\b[12] ), .b(\a[13] ), .o1(new_n165));
  norb02aa1n02x5               g070(.a(new_n165), .b(new_n164), .out0(new_n166));
  nano22aa1n02x4               g071(.a(new_n139), .b(new_n133), .c(new_n140), .out0(new_n167));
  aoi012aa1n02x5               g072(.a(new_n132), .b(\a[10] ), .c(\b[9] ), .o1(new_n168));
  nanp03aa1n02x5               g073(.a(new_n167), .b(new_n130), .c(new_n168), .o1(new_n169));
  nano22aa1n02x4               g074(.a(new_n166), .b(new_n169), .c(new_n160), .out0(new_n170));
  aoi022aa1n02x5               g075(.a(new_n163), .b(new_n166), .c(new_n148), .d(new_n170), .o1(\s[13] ));
  inv000aa1d42x5               g076(.a(new_n164), .o1(new_n172));
  aoai13aa1n06x5               g077(.a(new_n166), .b(new_n161), .c(new_n123), .d(new_n147), .o1(new_n173));
  nor002aa1n12x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nand02aa1d28x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  oai022aa1n02x5               g081(.a(\a[13] ), .b(\b[12] ), .c(\b[13] ), .d(\a[14] ), .o1(new_n177));
  nanb03aa1n02x5               g082(.a(new_n177), .b(new_n173), .c(new_n175), .out0(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n172), .d(new_n173), .o1(\s[14] ));
  nano23aa1n06x5               g084(.a(new_n164), .b(new_n174), .c(new_n175), .d(new_n165), .out0(new_n180));
  oaoi03aa1n02x5               g085(.a(\a[14] ), .b(\b[13] ), .c(new_n172), .o1(new_n181));
  nor042aa1n09x5               g086(.a(\b[14] ), .b(\a[15] ), .o1(new_n182));
  nanp02aa1n04x5               g087(.a(\b[14] ), .b(\a[15] ), .o1(new_n183));
  norb02aa1n03x5               g088(.a(new_n183), .b(new_n182), .out0(new_n184));
  aoai13aa1n06x5               g089(.a(new_n184), .b(new_n181), .c(new_n163), .d(new_n180), .o1(new_n185));
  aoi112aa1n02x5               g090(.a(new_n184), .b(new_n181), .c(new_n163), .d(new_n180), .o1(new_n186));
  norb02aa1n02x5               g091(.a(new_n185), .b(new_n186), .out0(\s[15] ));
  inv020aa1n02x5               g092(.a(new_n182), .o1(new_n188));
  nor042aa1d18x5               g093(.a(\b[15] ), .b(\a[16] ), .o1(new_n189));
  nanp02aa1n06x5               g094(.a(\b[15] ), .b(\a[16] ), .o1(new_n190));
  norb02aa1n06x4               g095(.a(new_n190), .b(new_n189), .out0(new_n191));
  nona23aa1n03x5               g096(.a(new_n185), .b(new_n190), .c(new_n189), .d(new_n182), .out0(new_n192));
  aoai13aa1n02x5               g097(.a(new_n192), .b(new_n191), .c(new_n188), .d(new_n185), .o1(\s[16] ));
  nand23aa1n03x5               g098(.a(new_n180), .b(new_n184), .c(new_n191), .o1(new_n194));
  nor042aa1n06x5               g099(.a(new_n194), .b(new_n146), .o1(new_n195));
  nanp02aa1n02x5               g100(.a(new_n123), .b(new_n195), .o1(new_n196));
  xnrc02aa1n02x5               g101(.a(\b[8] ), .b(\a[9] ), .out0(new_n197));
  xnrc02aa1n02x5               g102(.a(\b[9] ), .b(\a[10] ), .out0(new_n198));
  nona23aa1n02x5               g103(.a(new_n140), .b(new_n133), .c(new_n132), .d(new_n139), .out0(new_n199));
  nona23aa1n03x5               g104(.a(new_n175), .b(new_n165), .c(new_n164), .d(new_n174), .out0(new_n200));
  nona23aa1n03x5               g105(.a(new_n190), .b(new_n183), .c(new_n182), .d(new_n189), .out0(new_n201));
  nor042aa1n03x5               g106(.a(new_n201), .b(new_n200), .o1(new_n202));
  nona32aa1n03x5               g107(.a(new_n202), .b(new_n199), .c(new_n198), .d(new_n197), .out0(new_n203));
  nanb03aa1n03x5               g108(.a(new_n189), .b(new_n190), .c(new_n183), .out0(new_n204));
  oai112aa1n02x5               g109(.a(new_n188), .b(new_n175), .c(new_n174), .d(new_n164), .o1(new_n205));
  tech160nm_fiaoi012aa1n03p5x5 g110(.a(new_n189), .b(new_n182), .c(new_n190), .o1(new_n206));
  tech160nm_fioai012aa1n04x5   g111(.a(new_n206), .b(new_n205), .c(new_n204), .o1(new_n207));
  aoi012aa1n12x5               g112(.a(new_n207), .b(new_n161), .c(new_n202), .o1(new_n208));
  aoai13aa1n09x5               g113(.a(new_n208), .b(new_n203), .c(new_n157), .d(new_n122), .o1(new_n209));
  xorc02aa1n12x5               g114(.a(\a[17] ), .b(\b[16] ), .out0(new_n210));
  nano32aa1n03x7               g115(.a(new_n204), .b(new_n177), .c(new_n188), .d(new_n175), .out0(new_n211));
  nanb02aa1n02x5               g116(.a(new_n210), .b(new_n206), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(new_n212), .b(new_n211), .c(new_n161), .d(new_n202), .o1(new_n213));
  aoi022aa1n02x5               g118(.a(new_n209), .b(new_n210), .c(new_n196), .d(new_n213), .o1(\s[17] ));
  aobi12aa1n02x5               g119(.a(new_n210), .b(new_n196), .c(new_n208), .out0(new_n215));
  xorc02aa1n12x5               g120(.a(\a[18] ), .b(\b[17] ), .out0(new_n216));
  norp02aa1n02x5               g121(.a(\b[16] ), .b(\a[17] ), .o1(new_n217));
  aoi012aa1n02x5               g122(.a(new_n217), .b(new_n209), .c(new_n210), .o1(new_n218));
  oaih22aa1d12x5               g123(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n219));
  tech160nm_fiao0012aa1n02p5x5 g124(.a(new_n219), .b(\a[18] ), .c(\b[17] ), .o(new_n220));
  oai022aa1n02x5               g125(.a(new_n218), .b(new_n216), .c(new_n220), .d(new_n215), .o1(\s[18] ));
  norb02aa1n02x7               g126(.a(new_n206), .b(new_n211), .out0(new_n222));
  aoai13aa1n06x5               g127(.a(new_n222), .b(new_n194), .c(new_n169), .d(new_n160), .o1(new_n223));
  and002aa1n02x5               g128(.a(new_n216), .b(new_n210), .o(new_n224));
  aoai13aa1n06x5               g129(.a(new_n224), .b(new_n223), .c(new_n123), .d(new_n195), .o1(new_n225));
  aob012aa1n02x5               g130(.a(new_n219), .b(\b[17] ), .c(\a[18] ), .out0(new_n226));
  xorc02aa1n02x5               g131(.a(\a[19] ), .b(\b[18] ), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n225), .c(new_n226), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor002aa1d24x5               g134(.a(\b[18] ), .b(\a[19] ), .o1(new_n230));
  inv040aa1n06x5               g135(.a(new_n230), .o1(new_n231));
  aob012aa1n03x5               g136(.a(new_n227), .b(new_n225), .c(new_n226), .out0(new_n232));
  nor002aa1n06x5               g137(.a(\b[19] ), .b(\a[20] ), .o1(new_n233));
  and002aa1n12x5               g138(.a(\b[19] ), .b(\a[20] ), .o(new_n234));
  norp02aa1n02x5               g139(.a(new_n234), .b(new_n233), .o1(new_n235));
  norp03aa1n02x5               g140(.a(new_n234), .b(new_n233), .c(new_n230), .o1(new_n236));
  nanp02aa1n03x5               g141(.a(new_n232), .b(new_n236), .o1(new_n237));
  aoai13aa1n03x5               g142(.a(new_n237), .b(new_n235), .c(new_n232), .d(new_n231), .o1(\s[20] ));
  nand42aa1n08x5               g143(.a(\b[18] ), .b(\a[19] ), .o1(new_n239));
  nano23aa1n09x5               g144(.a(new_n234), .b(new_n233), .c(new_n231), .d(new_n239), .out0(new_n240));
  nand23aa1d12x5               g145(.a(new_n240), .b(new_n210), .c(new_n216), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n241), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n223), .c(new_n123), .d(new_n195), .o1(new_n243));
  tech160nm_fioai012aa1n04x5   g148(.a(new_n239), .b(\b[19] ), .c(\a[20] ), .o1(new_n244));
  aoi012aa1n06x5               g149(.a(new_n230), .b(\a[18] ), .c(\b[17] ), .o1(new_n245));
  nona23aa1d18x5               g150(.a(new_n219), .b(new_n245), .c(new_n244), .d(new_n234), .out0(new_n246));
  oab012aa1n09x5               g151(.a(new_n233), .b(new_n231), .c(new_n234), .out0(new_n247));
  and002aa1n02x5               g152(.a(new_n246), .b(new_n247), .o(new_n248));
  xorc02aa1n12x5               g153(.a(\a[21] ), .b(\b[20] ), .out0(new_n249));
  aob012aa1n03x5               g154(.a(new_n249), .b(new_n243), .c(new_n248), .out0(new_n250));
  inv000aa1d42x5               g155(.a(new_n249), .o1(new_n251));
  and003aa1n02x5               g156(.a(new_n246), .b(new_n251), .c(new_n247), .o(new_n252));
  aobi12aa1n02x7               g157(.a(new_n250), .b(new_n252), .c(new_n243), .out0(\s[21] ));
  norp02aa1n02x5               g158(.a(\b[20] ), .b(\a[21] ), .o1(new_n254));
  inv000aa1d42x5               g159(.a(new_n254), .o1(new_n255));
  tech160nm_fixorc02aa1n04x5   g160(.a(\a[22] ), .b(\b[21] ), .out0(new_n256));
  oai022aa1n02x5               g161(.a(\a[21] ), .b(\b[20] ), .c(\b[21] ), .d(\a[22] ), .o1(new_n257));
  aoi012aa1n02x5               g162(.a(new_n257), .b(\a[22] ), .c(\b[21] ), .o1(new_n258));
  aoai13aa1n02x5               g163(.a(new_n258), .b(new_n251), .c(new_n243), .d(new_n248), .o1(new_n259));
  aoai13aa1n03x5               g164(.a(new_n259), .b(new_n256), .c(new_n250), .d(new_n255), .o1(\s[22] ));
  nand22aa1n09x5               g165(.a(new_n256), .b(new_n249), .o1(new_n261));
  nano32aa1n02x4               g166(.a(new_n261), .b(new_n240), .c(new_n216), .d(new_n210), .out0(new_n262));
  inv000aa1d42x5               g167(.a(\b[21] ), .o1(new_n263));
  oaib12aa1n12x5               g168(.a(new_n257), .b(new_n263), .c(\a[22] ), .out0(new_n264));
  aoai13aa1n12x5               g169(.a(new_n264), .b(new_n261), .c(new_n246), .d(new_n247), .o1(new_n265));
  xorc02aa1n02x5               g170(.a(\a[23] ), .b(\b[22] ), .out0(new_n266));
  aoai13aa1n06x5               g171(.a(new_n266), .b(new_n265), .c(new_n209), .d(new_n262), .o1(new_n267));
  aoi012aa1n02x5               g172(.a(new_n261), .b(new_n246), .c(new_n247), .o1(new_n268));
  norb03aa1n02x5               g173(.a(new_n264), .b(new_n268), .c(new_n266), .out0(new_n269));
  aobi12aa1n02x5               g174(.a(new_n269), .b(new_n209), .c(new_n262), .out0(new_n270));
  norb02aa1n02x5               g175(.a(new_n267), .b(new_n270), .out0(\s[23] ));
  nor002aa1d24x5               g176(.a(\b[22] ), .b(\a[23] ), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n272), .o1(new_n273));
  xorc02aa1n02x5               g178(.a(\a[24] ), .b(\b[23] ), .out0(new_n274));
  and002aa1n02x5               g179(.a(\b[23] ), .b(\a[24] ), .o(new_n275));
  oai022aa1n02x5               g180(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n276));
  nona22aa1n03x5               g181(.a(new_n267), .b(new_n275), .c(new_n276), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n274), .c(new_n273), .d(new_n267), .o1(\s[24] ));
  inv000aa1d42x5               g183(.a(\b[20] ), .o1(new_n279));
  xroi22aa1d04x5               g184(.a(new_n263), .b(\a[22] ), .c(new_n279), .d(\a[21] ), .out0(new_n280));
  nanp02aa1n02x5               g185(.a(\b[22] ), .b(\a[23] ), .o1(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[23] ), .b(\a[24] ), .out0(new_n282));
  nano22aa1n03x7               g187(.a(new_n282), .b(new_n273), .c(new_n281), .out0(new_n283));
  nano22aa1n02x5               g188(.a(new_n241), .b(new_n280), .c(new_n283), .out0(new_n284));
  aoai13aa1n06x5               g189(.a(new_n284), .b(new_n223), .c(new_n123), .d(new_n195), .o1(new_n285));
  oaoi03aa1n02x5               g190(.a(\a[24] ), .b(\b[23] ), .c(new_n273), .o1(new_n286));
  aoi012aa1n02x5               g191(.a(new_n286), .b(new_n265), .c(new_n283), .o1(new_n287));
  nand22aa1n03x5               g192(.a(new_n285), .b(new_n287), .o1(new_n288));
  xorc02aa1n12x5               g193(.a(\a[25] ), .b(\b[24] ), .out0(new_n289));
  aoi112aa1n02x5               g194(.a(new_n289), .b(new_n286), .c(new_n265), .d(new_n283), .o1(new_n290));
  aoi022aa1n02x5               g195(.a(new_n288), .b(new_n289), .c(new_n285), .d(new_n290), .o1(\s[25] ));
  aobi12aa1n02x5               g196(.a(new_n289), .b(new_n285), .c(new_n287), .out0(new_n292));
  xorc02aa1n02x5               g197(.a(\a[26] ), .b(\b[25] ), .out0(new_n293));
  nor022aa1n04x5               g198(.a(\b[24] ), .b(\a[25] ), .o1(new_n294));
  tech160nm_fiaoi012aa1n05x5   g199(.a(new_n294), .b(new_n288), .c(new_n289), .o1(new_n295));
  nanp02aa1n02x5               g200(.a(\b[25] ), .b(\a[26] ), .o1(new_n296));
  inv000aa1d42x5               g201(.a(\a[26] ), .o1(new_n297));
  inv000aa1d42x5               g202(.a(\b[25] ), .o1(new_n298));
  aoi012aa1n02x5               g203(.a(new_n294), .b(new_n297), .c(new_n298), .o1(new_n299));
  nanp02aa1n02x5               g204(.a(new_n299), .b(new_n296), .o1(new_n300));
  oai022aa1n03x5               g205(.a(new_n295), .b(new_n293), .c(new_n300), .d(new_n292), .o1(\s[26] ));
  and002aa1n02x5               g206(.a(new_n293), .b(new_n289), .o(new_n302));
  aoai13aa1n09x5               g207(.a(new_n302), .b(new_n286), .c(new_n265), .d(new_n283), .o1(new_n303));
  nano32aa1n03x7               g208(.a(new_n241), .b(new_n302), .c(new_n280), .d(new_n283), .out0(new_n304));
  aoai13aa1n04x5               g209(.a(new_n304), .b(new_n223), .c(new_n123), .d(new_n195), .o1(new_n305));
  oaoi03aa1n02x5               g210(.a(new_n297), .b(new_n298), .c(new_n294), .o1(new_n306));
  nand23aa1n06x5               g211(.a(new_n305), .b(new_n303), .c(new_n306), .o1(new_n307));
  xorc02aa1n12x5               g212(.a(\a[27] ), .b(\b[26] ), .out0(new_n308));
  inv000aa1n02x5               g213(.a(new_n299), .o1(new_n309));
  aoi122aa1n02x7               g214(.a(new_n308), .b(new_n296), .c(new_n309), .d(new_n209), .e(new_n304), .o1(new_n310));
  aoi022aa1n03x5               g215(.a(new_n310), .b(new_n303), .c(new_n307), .d(new_n308), .o1(\s[27] ));
  norp02aa1n02x5               g216(.a(\b[26] ), .b(\a[27] ), .o1(new_n312));
  norp02aa1n02x5               g217(.a(\b[27] ), .b(\a[28] ), .o1(new_n313));
  and002aa1n03x5               g218(.a(\b[27] ), .b(\a[28] ), .o(new_n314));
  norp02aa1n02x5               g219(.a(new_n314), .b(new_n313), .o1(new_n315));
  inv040aa1n03x5               g220(.a(new_n315), .o1(new_n316));
  aoai13aa1n03x5               g221(.a(new_n316), .b(new_n312), .c(new_n307), .d(new_n308), .o1(new_n317));
  aoi022aa1n06x5               g222(.a(new_n209), .b(new_n304), .c(new_n296), .d(new_n309), .o1(new_n318));
  inv000aa1d42x5               g223(.a(new_n308), .o1(new_n319));
  norp03aa1n02x5               g224(.a(new_n314), .b(new_n313), .c(new_n312), .o1(new_n320));
  aoai13aa1n02x5               g225(.a(new_n320), .b(new_n319), .c(new_n318), .d(new_n303), .o1(new_n321));
  nanp02aa1n03x5               g226(.a(new_n317), .b(new_n321), .o1(\s[28] ));
  norb02aa1n02x5               g227(.a(new_n308), .b(new_n316), .out0(new_n323));
  nanp02aa1n03x5               g228(.a(new_n307), .b(new_n323), .o1(new_n324));
  inv000aa1n02x5               g229(.a(new_n323), .o1(new_n325));
  aoi112aa1n02x5               g230(.a(\b[26] ), .b(\a[27] ), .c(\a[28] ), .d(\b[27] ), .o1(new_n326));
  norp02aa1n02x5               g231(.a(new_n326), .b(new_n313), .o1(new_n327));
  aoai13aa1n03x5               g232(.a(new_n327), .b(new_n325), .c(new_n318), .d(new_n303), .o1(new_n328));
  xorc02aa1n02x5               g233(.a(\a[29] ), .b(\b[28] ), .out0(new_n329));
  norp03aa1n02x5               g234(.a(new_n329), .b(new_n326), .c(new_n313), .o1(new_n330));
  aoi022aa1n03x5               g235(.a(new_n328), .b(new_n329), .c(new_n324), .d(new_n330), .o1(\s[29] ));
  xorb03aa1n02x5               g236(.a(new_n149), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  inv000aa1n02x5               g237(.a(new_n329), .o1(new_n333));
  nona32aa1n03x5               g238(.a(new_n307), .b(new_n333), .c(new_n316), .d(new_n319), .out0(new_n334));
  nanp03aa1n02x5               g239(.a(new_n329), .b(new_n308), .c(new_n315), .o1(new_n335));
  inv000aa1d42x5               g240(.a(\a[29] ), .o1(new_n336));
  inv000aa1d42x5               g241(.a(\b[28] ), .o1(new_n337));
  aoi112aa1n02x5               g242(.a(new_n326), .b(new_n313), .c(new_n336), .d(new_n337), .o1(new_n338));
  oabi12aa1n02x5               g243(.a(new_n338), .b(new_n336), .c(new_n337), .out0(new_n339));
  aoai13aa1n02x5               g244(.a(new_n339), .b(new_n335), .c(new_n318), .d(new_n303), .o1(new_n340));
  xorc02aa1n02x5               g245(.a(\a[30] ), .b(\b[29] ), .out0(new_n341));
  norb02aa1n02x5               g246(.a(new_n339), .b(new_n341), .out0(new_n342));
  aoi022aa1n03x5               g247(.a(new_n340), .b(new_n341), .c(new_n334), .d(new_n342), .o1(\s[30] ));
  nanb02aa1n02x5               g248(.a(\a[31] ), .b(\b[30] ), .out0(new_n344));
  nanb02aa1n02x5               g249(.a(\b[30] ), .b(\a[31] ), .out0(new_n345));
  nanp02aa1n02x5               g250(.a(new_n345), .b(new_n344), .o1(new_n346));
  inv000aa1n02x5               g251(.a(new_n341), .o1(new_n347));
  nona32aa1n03x5               g252(.a(new_n307), .b(new_n347), .c(new_n333), .d(new_n325), .out0(new_n348));
  nona22aa1n02x4               g253(.a(new_n323), .b(new_n333), .c(new_n347), .out0(new_n349));
  norp02aa1n02x5               g254(.a(\b[29] ), .b(\a[30] ), .o1(new_n350));
  aoi122aa1n02x5               g255(.a(new_n338), .b(\b[29] ), .c(\a[30] ), .d(\b[28] ), .e(\a[29] ), .o1(new_n351));
  norp02aa1n02x5               g256(.a(new_n351), .b(new_n350), .o1(new_n352));
  aoai13aa1n02x5               g257(.a(new_n352), .b(new_n349), .c(new_n318), .d(new_n303), .o1(new_n353));
  nano23aa1n02x4               g258(.a(new_n351), .b(new_n350), .c(new_n344), .d(new_n345), .out0(new_n354));
  aoi022aa1n03x5               g259(.a(new_n353), .b(new_n346), .c(new_n348), .d(new_n354), .o1(\s[31] ));
  xobna2aa1n03x5               g260(.a(new_n103), .b(new_n100), .c(new_n99), .out0(\s[3] ));
  nanp02aa1n02x5               g261(.a(new_n104), .b(new_n105), .o1(new_n357));
  xorc02aa1n02x5               g262(.a(\a[4] ), .b(\b[3] ), .out0(new_n358));
  norp02aa1n02x5               g263(.a(new_n358), .b(new_n101), .o1(new_n359));
  aoi022aa1n02x5               g264(.a(new_n357), .b(new_n358), .c(new_n104), .d(new_n359), .o1(\s[4] ));
  xorc02aa1n02x5               g265(.a(\a[5] ), .b(\b[4] ), .out0(new_n361));
  xobna2aa1n03x5               g266(.a(new_n361), .b(new_n357), .c(new_n153), .out0(\s[5] ));
  inv000aa1d42x5               g267(.a(new_n120), .o1(new_n363));
  oai112aa1n02x5               g268(.a(new_n153), .b(new_n361), .c(new_n151), .d(new_n152), .o1(new_n364));
  norb02aa1n02x5               g269(.a(new_n108), .b(new_n119), .out0(new_n365));
  xnbna2aa1n03x5               g270(.a(new_n365), .b(new_n364), .c(new_n363), .out0(\s[6] ));
  inv000aa1d42x5               g271(.a(new_n119), .o1(new_n367));
  aob012aa1n03x5               g272(.a(new_n365), .b(new_n364), .c(new_n363), .out0(new_n368));
  xorc02aa1n02x5               g273(.a(\a[7] ), .b(\b[6] ), .out0(new_n369));
  xnbna2aa1n03x5               g274(.a(new_n369), .b(new_n368), .c(new_n367), .out0(\s[7] ));
  aob012aa1n03x5               g275(.a(new_n369), .b(new_n368), .c(new_n367), .out0(new_n371));
  nanp02aa1n02x5               g276(.a(new_n371), .b(new_n115), .o1(new_n372));
  norp02aa1n02x5               g277(.a(new_n117), .b(new_n106), .o1(new_n373));
  norb02aa1n02x5               g278(.a(new_n115), .b(new_n373), .out0(new_n374));
  aoi022aa1n02x7               g279(.a(new_n372), .b(new_n373), .c(new_n371), .d(new_n374), .o1(\s[8] ));
  aoi112aa1n02x5               g280(.a(new_n124), .b(new_n116), .c(new_n121), .d(new_n118), .o1(new_n376));
  aoi022aa1n02x5               g281(.a(new_n123), .b(new_n124), .c(new_n157), .d(new_n376), .o1(\s[9] ));
endmodule


