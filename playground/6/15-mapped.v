// Benchmark "adder" written by ABC on Wed Jul 17 15:06:30 2024

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
    new_n132, new_n133, new_n134, new_n135, new_n136, new_n137, new_n138,
    new_n139, new_n141, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n148, new_n149, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n172, new_n173, new_n174, new_n175, new_n176, new_n177,
    new_n178, new_n180, new_n181, new_n182, new_n183, new_n184, new_n186,
    new_n187, new_n188, new_n189, new_n190, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n213, new_n214, new_n215, new_n216,
    new_n217, new_n218, new_n219, new_n220, new_n221, new_n223, new_n224,
    new_n225, new_n226, new_n227, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n256,
    new_n257, new_n258, new_n259, new_n260, new_n261, new_n262, new_n263,
    new_n265, new_n266, new_n267, new_n268, new_n269, new_n270, new_n271,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n283, new_n284, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n301, new_n302,
    new_n303, new_n304, new_n305, new_n306, new_n307, new_n308, new_n310,
    new_n311, new_n312, new_n313, new_n314, new_n315, new_n316, new_n317,
    new_n318, new_n320, new_n321, new_n322, new_n323, new_n324, new_n325,
    new_n326, new_n327, new_n328, new_n330, new_n331, new_n332, new_n333,
    new_n334, new_n335, new_n336, new_n337, new_n338, new_n341, new_n342,
    new_n343, new_n344, new_n345, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n353, new_n354, new_n355, new_n356, new_n357,
    new_n358, new_n359, new_n362, new_n363, new_n365, new_n367, new_n368,
    new_n369, new_n371, new_n372, new_n375;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[2] ), .b(\b[1] ), .o(new_n97));
  nanp02aa1n02x5               g002(.a(\b[0] ), .b(\a[1] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp03aa1n02x5               g004(.a(new_n97), .b(new_n98), .c(new_n99), .o1(new_n100));
  nor042aa1n04x5               g005(.a(\b[3] ), .b(\a[4] ), .o1(new_n101));
  nanp02aa1n02x5               g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor002aa1d24x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nona23aa1n02x4               g009(.a(new_n104), .b(new_n102), .c(new_n101), .d(new_n103), .out0(new_n105));
  tech160nm_fioai012aa1n02p5x5 g010(.a(new_n102), .b(new_n103), .c(new_n101), .o1(new_n106));
  aoai13aa1n06x5               g011(.a(new_n106), .b(new_n105), .c(new_n100), .d(new_n97), .o1(new_n107));
  aoi022aa1n06x5               g012(.a(\b[6] ), .b(\a[7] ), .c(\a[5] ), .d(\b[4] ), .o1(new_n108));
  oai122aa1n03x5               g013(.a(new_n108), .b(\a[6] ), .c(\b[5] ), .d(\a[5] ), .e(\b[4] ), .o1(new_n109));
  tech160nm_fixorc02aa1n04x5   g014(.a(\a[8] ), .b(\b[7] ), .out0(new_n110));
  nand42aa1n04x5               g015(.a(\b[5] ), .b(\a[6] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n111), .b(\b[6] ), .c(\a[7] ), .o1(new_n112));
  norb03aa1n03x5               g017(.a(new_n110), .b(new_n109), .c(new_n112), .out0(new_n113));
  nanp02aa1n02x5               g018(.a(new_n107), .b(new_n113), .o1(new_n114));
  orn002aa1n12x5               g019(.a(\a[5] ), .b(\b[4] ), .o(new_n115));
  norp02aa1n02x5               g020(.a(\b[5] ), .b(\a[6] ), .o1(new_n116));
  nanb03aa1n06x5               g021(.a(new_n116), .b(new_n115), .c(new_n111), .out0(new_n117));
  nanp02aa1n02x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  nor042aa1n06x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nano22aa1n06x5               g024(.a(new_n119), .b(new_n118), .c(new_n111), .out0(new_n120));
  inv040aa1n03x5               g025(.a(new_n119), .o1(new_n121));
  oaoi03aa1n12x5               g026(.a(\a[8] ), .b(\b[7] ), .c(new_n121), .o1(new_n122));
  aoi013aa1n06x4               g027(.a(new_n122), .b(new_n117), .c(new_n120), .d(new_n110), .o1(new_n123));
  nor042aa1n02x5               g028(.a(\b[8] ), .b(\a[9] ), .o1(new_n124));
  nand42aa1n02x5               g029(.a(\b[8] ), .b(\a[9] ), .o1(new_n125));
  norb02aa1n02x5               g030(.a(new_n125), .b(new_n124), .out0(new_n126));
  aobi12aa1n02x5               g031(.a(new_n126), .b(new_n114), .c(new_n123), .out0(new_n127));
  aob012aa1n02x5               g032(.a(new_n97), .b(new_n98), .c(new_n99), .out0(new_n128));
  norb02aa1n02x5               g033(.a(new_n102), .b(new_n101), .out0(new_n129));
  norb02aa1n02x5               g034(.a(new_n104), .b(new_n103), .out0(new_n130));
  nand23aa1n03x5               g035(.a(new_n128), .b(new_n129), .c(new_n130), .o1(new_n131));
  nona22aa1n03x5               g036(.a(new_n110), .b(new_n109), .c(new_n112), .out0(new_n132));
  aoai13aa1n12x5               g037(.a(new_n123), .b(new_n132), .c(new_n131), .d(new_n106), .o1(new_n133));
  aoi012aa1n02x5               g038(.a(new_n124), .b(new_n133), .c(new_n125), .o1(new_n134));
  nor002aa1n03x5               g039(.a(\b[9] ), .b(\a[10] ), .o1(new_n135));
  nanp02aa1n04x5               g040(.a(\b[9] ), .b(\a[10] ), .o1(new_n136));
  norb02aa1n02x5               g041(.a(new_n136), .b(new_n135), .out0(new_n137));
  norb03aa1n06x5               g042(.a(new_n136), .b(new_n124), .c(new_n135), .out0(new_n138));
  inv040aa1n03x5               g043(.a(new_n138), .o1(new_n139));
  oai022aa1n02x5               g044(.a(new_n134), .b(new_n137), .c(new_n139), .d(new_n127), .o1(\s[10] ));
  nanp03aa1n02x5               g045(.a(new_n117), .b(new_n120), .c(new_n110), .o1(new_n141));
  inv000aa1d42x5               g046(.a(new_n122), .o1(new_n142));
  nanp02aa1n02x5               g047(.a(new_n141), .b(new_n142), .o1(new_n143));
  nano23aa1n06x5               g048(.a(new_n124), .b(new_n135), .c(new_n136), .d(new_n125), .out0(new_n144));
  aoai13aa1n02x5               g049(.a(new_n144), .b(new_n143), .c(new_n107), .d(new_n113), .o1(new_n145));
  oai012aa1n02x5               g050(.a(new_n136), .b(new_n135), .c(new_n124), .o1(new_n146));
  nor042aa1n06x5               g051(.a(\b[10] ), .b(\a[11] ), .o1(new_n147));
  nand42aa1n03x5               g052(.a(\b[10] ), .b(\a[11] ), .o1(new_n148));
  norb02aa1n03x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n145), .c(new_n146), .out0(\s[11] ));
  inv000aa1d42x5               g055(.a(new_n147), .o1(new_n151));
  aob012aa1n02x5               g056(.a(new_n149), .b(new_n145), .c(new_n146), .out0(new_n152));
  norp02aa1n04x5               g057(.a(\b[11] ), .b(\a[12] ), .o1(new_n153));
  nand42aa1n06x5               g058(.a(\b[11] ), .b(\a[12] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  nona23aa1n02x4               g060(.a(new_n152), .b(new_n154), .c(new_n153), .d(new_n147), .out0(new_n156));
  aoai13aa1n02x5               g061(.a(new_n156), .b(new_n155), .c(new_n152), .d(new_n151), .o1(\s[12] ));
  and003aa1n02x5               g062(.a(new_n144), .b(new_n155), .c(new_n149), .o(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n143), .c(new_n107), .d(new_n113), .o1(new_n159));
  oai012aa1n04x7               g064(.a(new_n136), .b(\b[10] ), .c(\a[11] ), .o1(new_n160));
  nanb03aa1n09x5               g065(.a(new_n153), .b(new_n154), .c(new_n148), .out0(new_n161));
  oai012aa1n06x5               g066(.a(new_n154), .b(new_n153), .c(new_n147), .o1(new_n162));
  oai013aa1d12x5               g067(.a(new_n162), .b(new_n138), .c(new_n161), .d(new_n160), .o1(new_n163));
  inv000aa1d42x5               g068(.a(new_n163), .o1(new_n164));
  nand42aa1n02x5               g069(.a(new_n159), .b(new_n164), .o1(new_n165));
  nor002aa1n12x5               g070(.a(\b[12] ), .b(\a[13] ), .o1(new_n166));
  nand02aa1n03x5               g071(.a(\b[12] ), .b(\a[13] ), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(new_n168));
  nona22aa1n09x5               g073(.a(new_n139), .b(new_n161), .c(new_n160), .out0(new_n169));
  nano22aa1n02x4               g074(.a(new_n168), .b(new_n169), .c(new_n162), .out0(new_n170));
  aoi022aa1n02x5               g075(.a(new_n165), .b(new_n168), .c(new_n159), .d(new_n170), .o1(\s[13] ));
  inv000aa1d42x5               g076(.a(new_n166), .o1(new_n172));
  aoai13aa1n02x5               g077(.a(new_n168), .b(new_n163), .c(new_n133), .d(new_n158), .o1(new_n173));
  nor002aa1n03x5               g078(.a(\b[13] ), .b(\a[14] ), .o1(new_n174));
  nanp02aa1n04x5               g079(.a(\b[13] ), .b(\a[14] ), .o1(new_n175));
  norb02aa1n02x5               g080(.a(new_n175), .b(new_n174), .out0(new_n176));
  norb03aa1n06x5               g081(.a(new_n175), .b(new_n166), .c(new_n174), .out0(new_n177));
  nanp02aa1n02x5               g082(.a(new_n173), .b(new_n177), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n178), .b(new_n176), .c(new_n172), .d(new_n173), .o1(\s[14] ));
  oaoi03aa1n02x5               g084(.a(\a[14] ), .b(\b[13] ), .c(new_n172), .o1(new_n180));
  inv000aa1d42x5               g085(.a(new_n180), .o1(new_n181));
  nano23aa1n06x5               g086(.a(new_n166), .b(new_n174), .c(new_n175), .d(new_n167), .out0(new_n182));
  aoai13aa1n06x5               g087(.a(new_n182), .b(new_n163), .c(new_n133), .d(new_n158), .o1(new_n183));
  xorc02aa1n12x5               g088(.a(\a[15] ), .b(\b[14] ), .out0(new_n184));
  xnbna2aa1n03x5               g089(.a(new_n184), .b(new_n183), .c(new_n181), .out0(\s[15] ));
  nor002aa1n06x5               g090(.a(\b[14] ), .b(\a[15] ), .o1(new_n186));
  inv000aa1d42x5               g091(.a(new_n186), .o1(new_n187));
  aoai13aa1n02x5               g092(.a(new_n184), .b(new_n180), .c(new_n165), .d(new_n182), .o1(new_n188));
  xorc02aa1n03x5               g093(.a(\a[16] ), .b(\b[15] ), .out0(new_n189));
  inv000aa1d42x5               g094(.a(new_n184), .o1(new_n190));
  inv000aa1d42x5               g095(.a(\a[16] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(\b[15] ), .o1(new_n192));
  nanp02aa1n02x5               g097(.a(new_n192), .b(new_n191), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[15] ), .b(\a[16] ), .o1(new_n194));
  nano22aa1n02x4               g099(.a(new_n186), .b(new_n193), .c(new_n194), .out0(new_n195));
  aoai13aa1n02x5               g100(.a(new_n195), .b(new_n190), .c(new_n183), .d(new_n181), .o1(new_n196));
  aoai13aa1n03x5               g101(.a(new_n196), .b(new_n189), .c(new_n188), .d(new_n187), .o1(\s[16] ));
  nand23aa1n06x5               g102(.a(new_n182), .b(new_n184), .c(new_n189), .o1(new_n198));
  nano32aa1d12x5               g103(.a(new_n198), .b(new_n155), .c(new_n144), .d(new_n149), .out0(new_n199));
  aoai13aa1n06x5               g104(.a(new_n199), .b(new_n143), .c(new_n107), .d(new_n113), .o1(new_n200));
  oai012aa1n02x5               g105(.a(new_n175), .b(\b[14] ), .c(\a[15] ), .o1(new_n201));
  nanp02aa1n02x5               g106(.a(\b[14] ), .b(\a[15] ), .o1(new_n202));
  nanp03aa1n02x5               g107(.a(new_n193), .b(new_n202), .c(new_n194), .o1(new_n203));
  oaoi03aa1n02x5               g108(.a(new_n191), .b(new_n192), .c(new_n186), .o1(new_n204));
  oai013aa1n03x5               g109(.a(new_n204), .b(new_n177), .c(new_n203), .d(new_n201), .o1(new_n205));
  aoib12aa1n06x5               g110(.a(new_n205), .b(new_n163), .c(new_n198), .out0(new_n206));
  nanp02aa1n06x5               g111(.a(new_n200), .b(new_n206), .o1(new_n207));
  tech160nm_fixorc02aa1n03p5x5 g112(.a(\a[17] ), .b(\b[16] ), .out0(new_n208));
  norb02aa1n02x5               g113(.a(new_n204), .b(new_n208), .out0(new_n209));
  oai013aa1n02x4               g114(.a(new_n209), .b(new_n201), .c(new_n177), .d(new_n203), .o1(new_n210));
  aoib12aa1n02x5               g115(.a(new_n210), .b(new_n163), .c(new_n198), .out0(new_n211));
  aoi022aa1n02x5               g116(.a(new_n207), .b(new_n208), .c(new_n200), .d(new_n211), .o1(\s[17] ));
  inv000aa1d42x5               g117(.a(\a[17] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(\b[16] ), .b(new_n213), .out0(new_n214));
  inv000aa1n02x5               g119(.a(new_n205), .o1(new_n215));
  aoai13aa1n09x5               g120(.a(new_n215), .b(new_n198), .c(new_n169), .d(new_n162), .o1(new_n216));
  aoai13aa1n02x5               g121(.a(new_n208), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n217));
  xorc02aa1n02x5               g122(.a(\a[18] ), .b(\b[17] ), .out0(new_n218));
  and002aa1n02x5               g123(.a(\b[17] ), .b(\a[18] ), .o(new_n219));
  oaih22aa1n04x5               g124(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n220));
  nona22aa1n02x4               g125(.a(new_n217), .b(new_n219), .c(new_n220), .out0(new_n221));
  aoai13aa1n02x5               g126(.a(new_n221), .b(new_n218), .c(new_n214), .d(new_n217), .o1(\s[18] ));
  and002aa1n02x5               g127(.a(new_n218), .b(new_n208), .o(new_n223));
  aoai13aa1n06x5               g128(.a(new_n223), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n224));
  oaoi03aa1n02x5               g129(.a(\a[18] ), .b(\b[17] ), .c(new_n214), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  tech160nm_fixorc02aa1n03p5x5 g131(.a(\a[19] ), .b(\b[18] ), .out0(new_n227));
  xnbna2aa1n03x5               g132(.a(new_n227), .b(new_n224), .c(new_n226), .out0(\s[19] ));
  xnrc02aa1n02x5               g133(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv040aa1d32x5               g134(.a(\a[19] ), .o1(new_n230));
  inv000aa1d42x5               g135(.a(\b[18] ), .o1(new_n231));
  nanp02aa1n02x5               g136(.a(new_n231), .b(new_n230), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n227), .b(new_n225), .c(new_n207), .d(new_n223), .o1(new_n233));
  nor042aa1n03x5               g138(.a(\b[19] ), .b(\a[20] ), .o1(new_n234));
  nand02aa1n06x5               g139(.a(\b[19] ), .b(\a[20] ), .o1(new_n235));
  norb02aa1n02x5               g140(.a(new_n235), .b(new_n234), .out0(new_n236));
  inv000aa1d42x5               g141(.a(new_n227), .o1(new_n237));
  nano22aa1n02x4               g142(.a(new_n234), .b(new_n232), .c(new_n235), .out0(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n237), .c(new_n224), .d(new_n226), .o1(new_n239));
  aoai13aa1n03x5               g144(.a(new_n239), .b(new_n236), .c(new_n233), .d(new_n232), .o1(\s[20] ));
  nano32aa1n03x7               g145(.a(new_n237), .b(new_n218), .c(new_n208), .d(new_n236), .out0(new_n241));
  aoai13aa1n06x5               g146(.a(new_n241), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n242));
  aoi022aa1n02x7               g147(.a(new_n231), .b(new_n230), .c(\a[18] ), .d(\b[17] ), .o1(new_n243));
  nand42aa1n02x5               g148(.a(\b[18] ), .b(\a[19] ), .o1(new_n244));
  nano22aa1n03x7               g149(.a(new_n234), .b(new_n244), .c(new_n235), .out0(new_n245));
  oai112aa1n06x5               g150(.a(new_n245), .b(new_n243), .c(new_n220), .d(new_n219), .o1(new_n246));
  aoai13aa1n04x5               g151(.a(new_n235), .b(new_n234), .c(new_n230), .d(new_n231), .o1(new_n247));
  nand02aa1d06x5               g152(.a(new_n246), .b(new_n247), .o1(new_n248));
  nor002aa1d32x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  nand02aa1d28x5               g154(.a(\b[20] ), .b(\a[21] ), .o1(new_n250));
  norb02aa1n02x5               g155(.a(new_n250), .b(new_n249), .out0(new_n251));
  aoai13aa1n06x5               g156(.a(new_n251), .b(new_n248), .c(new_n207), .d(new_n241), .o1(new_n252));
  inv000aa1d42x5               g157(.a(new_n251), .o1(new_n253));
  and003aa1n02x5               g158(.a(new_n246), .b(new_n253), .c(new_n247), .o(new_n254));
  aobi12aa1n02x7               g159(.a(new_n252), .b(new_n254), .c(new_n242), .out0(\s[21] ));
  inv000aa1d42x5               g160(.a(new_n249), .o1(new_n256));
  nor002aa1d32x5               g161(.a(\b[21] ), .b(\a[22] ), .o1(new_n257));
  nand42aa1d28x5               g162(.a(\b[21] ), .b(\a[22] ), .o1(new_n258));
  norb02aa1n02x5               g163(.a(new_n258), .b(new_n257), .out0(new_n259));
  inv000aa1d42x5               g164(.a(new_n248), .o1(new_n260));
  nona22aa1d36x5               g165(.a(new_n258), .b(new_n257), .c(new_n249), .out0(new_n261));
  inv000aa1d42x5               g166(.a(new_n261), .o1(new_n262));
  aoai13aa1n03x5               g167(.a(new_n262), .b(new_n253), .c(new_n242), .d(new_n260), .o1(new_n263));
  aoai13aa1n03x5               g168(.a(new_n263), .b(new_n259), .c(new_n252), .d(new_n256), .o1(\s[22] ));
  nano32aa1n02x4               g169(.a(new_n234), .b(new_n232), .c(new_n235), .d(new_n244), .out0(new_n265));
  nano23aa1d15x5               g170(.a(new_n249), .b(new_n257), .c(new_n258), .d(new_n250), .out0(new_n266));
  inv000aa1d42x5               g171(.a(new_n266), .o1(new_n267));
  nano32aa1n02x4               g172(.a(new_n267), .b(new_n265), .c(new_n218), .d(new_n208), .out0(new_n268));
  aoai13aa1n03x5               g173(.a(new_n268), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n269));
  aoi022aa1n02x5               g174(.a(new_n248), .b(new_n266), .c(new_n258), .d(new_n261), .o1(new_n270));
  inv040aa1n03x5               g175(.a(new_n270), .o1(new_n271));
  nor002aa1n20x5               g176(.a(\b[22] ), .b(\a[23] ), .o1(new_n272));
  nand22aa1n06x5               g177(.a(\b[22] ), .b(\a[23] ), .o1(new_n273));
  norb02aa1n09x5               g178(.a(new_n273), .b(new_n272), .out0(new_n274));
  aoai13aa1n06x5               g179(.a(new_n274), .b(new_n271), .c(new_n207), .d(new_n268), .o1(new_n275));
  aoi122aa1n02x5               g180(.a(new_n274), .b(new_n258), .c(new_n261), .d(new_n248), .e(new_n266), .o1(new_n276));
  aobi12aa1n02x5               g181(.a(new_n275), .b(new_n276), .c(new_n269), .out0(\s[23] ));
  inv000aa1d42x5               g182(.a(new_n272), .o1(new_n278));
  tech160nm_fixorc02aa1n04x5   g183(.a(\a[24] ), .b(\b[23] ), .out0(new_n279));
  inv000aa1d42x5               g184(.a(new_n274), .o1(new_n280));
  and002aa1n12x5               g185(.a(\b[23] ), .b(\a[24] ), .o(new_n281));
  oai022aa1n09x5               g186(.a(\a[23] ), .b(\b[22] ), .c(\b[23] ), .d(\a[24] ), .o1(new_n282));
  norp02aa1n02x5               g187(.a(new_n282), .b(new_n281), .o1(new_n283));
  aoai13aa1n03x5               g188(.a(new_n283), .b(new_n280), .c(new_n269), .d(new_n270), .o1(new_n284));
  aoai13aa1n02x5               g189(.a(new_n284), .b(new_n279), .c(new_n275), .d(new_n278), .o1(\s[24] ));
  nand23aa1d12x5               g190(.a(new_n266), .b(new_n274), .c(new_n279), .o1(new_n286));
  nano32aa1n02x4               g191(.a(new_n286), .b(new_n265), .c(new_n218), .d(new_n208), .out0(new_n287));
  aoai13aa1n03x5               g192(.a(new_n287), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n281), .o1(new_n289));
  tech160nm_fioai012aa1n04x5   g194(.a(new_n258), .b(\b[22] ), .c(\a[23] ), .o1(new_n290));
  oai012aa1n03x5               g195(.a(new_n273), .b(\b[23] ), .c(\a[24] ), .o1(new_n291));
  nor003aa1n04x5               g196(.a(new_n291), .b(new_n290), .c(new_n281), .o1(new_n292));
  aoi022aa1d24x5               g197(.a(new_n292), .b(new_n261), .c(new_n289), .d(new_n282), .o1(new_n293));
  aoai13aa1n12x5               g198(.a(new_n293), .b(new_n286), .c(new_n246), .d(new_n247), .o1(new_n294));
  xorc02aa1n12x5               g199(.a(\a[25] ), .b(\b[24] ), .out0(new_n295));
  aoai13aa1n06x5               g200(.a(new_n295), .b(new_n294), .c(new_n207), .d(new_n287), .o1(new_n296));
  inv000aa1n06x5               g201(.a(new_n286), .o1(new_n297));
  aoi122aa1n02x5               g202(.a(new_n295), .b(new_n289), .c(new_n282), .d(new_n292), .e(new_n261), .o1(new_n298));
  aobi12aa1n02x5               g203(.a(new_n298), .b(new_n297), .c(new_n248), .out0(new_n299));
  aobi12aa1n02x5               g204(.a(new_n296), .b(new_n299), .c(new_n288), .out0(\s[25] ));
  orn002aa1n02x5               g205(.a(\a[25] ), .b(\b[24] ), .o(new_n301));
  xorc02aa1n02x5               g206(.a(\a[26] ), .b(\b[25] ), .out0(new_n302));
  inv000aa1d42x5               g207(.a(new_n294), .o1(new_n303));
  inv000aa1d42x5               g208(.a(new_n295), .o1(new_n304));
  and002aa1n02x5               g209(.a(\b[25] ), .b(\a[26] ), .o(new_n305));
  oai022aa1n02x5               g210(.a(\a[25] ), .b(\b[24] ), .c(\b[25] ), .d(\a[26] ), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n306), .b(new_n305), .o1(new_n307));
  aoai13aa1n03x5               g212(.a(new_n307), .b(new_n304), .c(new_n288), .d(new_n303), .o1(new_n308));
  aoai13aa1n03x5               g213(.a(new_n308), .b(new_n302), .c(new_n296), .d(new_n301), .o1(\s[26] ));
  and002aa1n02x5               g214(.a(new_n295), .b(new_n302), .o(new_n310));
  nand23aa1n04x5               g215(.a(new_n297), .b(new_n241), .c(new_n310), .o1(new_n311));
  inv000aa1n02x5               g216(.a(new_n311), .o1(new_n312));
  aoai13aa1n06x5               g217(.a(new_n312), .b(new_n216), .c(new_n133), .d(new_n199), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n305), .o1(new_n314));
  aoi022aa1n12x5               g219(.a(new_n294), .b(new_n310), .c(new_n314), .d(new_n306), .o1(new_n315));
  aoai13aa1n06x5               g220(.a(new_n315), .b(new_n311), .c(new_n200), .d(new_n206), .o1(new_n316));
  xorc02aa1n12x5               g221(.a(\a[27] ), .b(\b[26] ), .out0(new_n317));
  aoi122aa1n02x5               g222(.a(new_n317), .b(new_n314), .c(new_n306), .d(new_n294), .e(new_n310), .o1(new_n318));
  aoi022aa1n02x5               g223(.a(new_n316), .b(new_n317), .c(new_n313), .d(new_n318), .o1(\s[27] ));
  norp02aa1n02x5               g224(.a(\b[26] ), .b(\a[27] ), .o1(new_n320));
  inv000aa1d42x5               g225(.a(new_n320), .o1(new_n321));
  nanp02aa1n03x5               g226(.a(new_n316), .b(new_n317), .o1(new_n322));
  xorc02aa1n02x5               g227(.a(\a[28] ), .b(\b[27] ), .out0(new_n323));
  inv000aa1d42x5               g228(.a(new_n317), .o1(new_n324));
  nand22aa1n03x5               g229(.a(\b[27] ), .b(\a[28] ), .o1(new_n325));
  oai022aa1d24x5               g230(.a(\a[27] ), .b(\b[26] ), .c(\b[27] ), .d(\a[28] ), .o1(new_n326));
  norb02aa1n06x4               g231(.a(new_n325), .b(new_n326), .out0(new_n327));
  aoai13aa1n02x5               g232(.a(new_n327), .b(new_n324), .c(new_n313), .d(new_n315), .o1(new_n328));
  aoai13aa1n03x5               g233(.a(new_n328), .b(new_n323), .c(new_n322), .d(new_n321), .o1(\s[28] ));
  nor042aa1n02x5               g234(.a(\b[28] ), .b(\a[29] ), .o1(new_n330));
  nanp02aa1n02x5               g235(.a(\b[28] ), .b(\a[29] ), .o1(new_n331));
  nanb02aa1n02x5               g236(.a(new_n330), .b(new_n331), .out0(new_n332));
  and002aa1n02x5               g237(.a(new_n323), .b(new_n317), .o(new_n333));
  oaoi03aa1n02x5               g238(.a(\a[28] ), .b(\b[27] ), .c(new_n321), .o1(new_n334));
  aoai13aa1n03x5               g239(.a(new_n332), .b(new_n334), .c(new_n316), .d(new_n333), .o1(new_n335));
  inv000aa1d42x5               g240(.a(new_n333), .o1(new_n336));
  aoi012aa1n02x5               g241(.a(new_n332), .b(new_n325), .c(new_n326), .o1(new_n337));
  aoai13aa1n02x5               g242(.a(new_n337), .b(new_n336), .c(new_n313), .d(new_n315), .o1(new_n338));
  nanp02aa1n03x5               g243(.a(new_n335), .b(new_n338), .o1(\s[29] ));
  xorb03aa1n02x5               g244(.a(new_n98), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g245(.a(new_n332), .b(new_n317), .c(new_n323), .out0(new_n341));
  nanp02aa1n03x5               g246(.a(new_n316), .b(new_n341), .o1(new_n342));
  aoi013aa1n02x4               g247(.a(new_n330), .b(new_n326), .c(new_n325), .d(new_n331), .o1(new_n343));
  norp02aa1n02x5               g248(.a(\b[29] ), .b(\a[30] ), .o1(new_n344));
  nanp02aa1n02x5               g249(.a(\b[29] ), .b(\a[30] ), .o1(new_n345));
  norb02aa1n02x5               g250(.a(new_n345), .b(new_n344), .out0(new_n346));
  inv000aa1n02x5               g251(.a(new_n341), .o1(new_n347));
  nona23aa1n03x5               g252(.a(new_n331), .b(new_n325), .c(new_n327), .d(new_n330), .out0(new_n348));
  nona23aa1n06x5               g253(.a(new_n348), .b(new_n345), .c(new_n344), .d(new_n330), .out0(new_n349));
  inv000aa1d42x5               g254(.a(new_n349), .o1(new_n350));
  aoai13aa1n02x5               g255(.a(new_n350), .b(new_n347), .c(new_n313), .d(new_n315), .o1(new_n351));
  aoai13aa1n03x5               g256(.a(new_n351), .b(new_n346), .c(new_n342), .d(new_n343), .o1(\s[30] ));
  nano32aa1d12x5               g257(.a(new_n332), .b(new_n323), .c(new_n317), .d(new_n346), .out0(new_n353));
  and002aa1n02x5               g258(.a(new_n349), .b(new_n345), .o(new_n354));
  xnrc02aa1n02x5               g259(.a(\b[30] ), .b(\a[31] ), .out0(new_n355));
  aoai13aa1n03x5               g260(.a(new_n355), .b(new_n354), .c(new_n316), .d(new_n353), .o1(new_n356));
  inv000aa1d42x5               g261(.a(new_n353), .o1(new_n357));
  aoi012aa1n02x5               g262(.a(new_n355), .b(new_n349), .c(new_n345), .o1(new_n358));
  aoai13aa1n02x5               g263(.a(new_n358), .b(new_n357), .c(new_n313), .d(new_n315), .o1(new_n359));
  nanp02aa1n03x5               g264(.a(new_n356), .b(new_n359), .o1(\s[31] ));
  xnbna2aa1n03x5               g265(.a(new_n130), .b(new_n100), .c(new_n97), .out0(\s[3] ));
  inv000aa1d42x5               g266(.a(new_n103), .o1(new_n362));
  nanp02aa1n02x5               g267(.a(new_n128), .b(new_n130), .o1(new_n363));
  xnbna2aa1n03x5               g268(.a(new_n129), .b(new_n363), .c(new_n362), .out0(\s[4] ));
  xorc02aa1n02x5               g269(.a(\a[5] ), .b(\b[4] ), .out0(new_n365));
  xnbna2aa1n03x5               g270(.a(new_n365), .b(new_n131), .c(new_n106), .out0(\s[5] ));
  nanp02aa1n02x5               g271(.a(new_n107), .b(new_n365), .o1(new_n367));
  norb02aa1n02x5               g272(.a(new_n111), .b(new_n116), .out0(new_n368));
  nanb02aa1n02x5               g273(.a(new_n117), .b(new_n367), .out0(new_n369));
  aoai13aa1n02x5               g274(.a(new_n369), .b(new_n368), .c(new_n115), .d(new_n367), .o1(\s[6] ));
  aoai13aa1n02x5               g275(.a(new_n120), .b(new_n117), .c(new_n107), .d(new_n365), .o1(new_n371));
  aoi022aa1n02x5               g276(.a(new_n369), .b(new_n111), .c(new_n118), .d(new_n121), .o1(new_n372));
  norb02aa1n02x5               g277(.a(new_n371), .b(new_n372), .out0(\s[7] ));
  xnbna2aa1n03x5               g278(.a(new_n110), .b(new_n371), .c(new_n121), .out0(\s[8] ));
  aoi113aa1n02x5               g279(.a(new_n126), .b(new_n122), .c(new_n117), .d(new_n120), .e(new_n110), .o1(new_n375));
  aoi022aa1n02x5               g280(.a(new_n133), .b(new_n126), .c(new_n114), .d(new_n375), .o1(\s[9] ));
endmodule

