// Benchmark "adder" written by ABC on Wed Jul 17 13:54:52 2024

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
    new_n132, new_n133, new_n135, new_n136, new_n137, new_n139, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n166, new_n167, new_n168, new_n169, new_n170,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n177, new_n178,
    new_n179, new_n181, new_n182, new_n183, new_n184, new_n185, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n196, new_n197, new_n198, new_n199, new_n200, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n209,
    new_n210, new_n211, new_n212, new_n213, new_n214, new_n215, new_n217,
    new_n218, new_n219, new_n220, new_n221, new_n222, new_n223, new_n226,
    new_n227, new_n228, new_n229, new_n230, new_n231, new_n232, new_n233,
    new_n234, new_n236, new_n237, new_n238, new_n239, new_n240, new_n241,
    new_n242, new_n243, new_n244, new_n245, new_n246, new_n248, new_n249,
    new_n250, new_n251, new_n252, new_n254, new_n255, new_n256, new_n257,
    new_n258, new_n259, new_n260, new_n261, new_n262, new_n264, new_n265,
    new_n266, new_n267, new_n268, new_n269, new_n271, new_n272, new_n273,
    new_n274, new_n275, new_n276, new_n277, new_n278, new_n279, new_n280,
    new_n281, new_n283, new_n284, new_n285, new_n286, new_n287, new_n288,
    new_n290, new_n291, new_n292, new_n293, new_n294, new_n295, new_n296,
    new_n297, new_n298, new_n300, new_n301, new_n302, new_n303, new_n304,
    new_n305, new_n306, new_n307, new_n308, new_n309, new_n310, new_n311,
    new_n313, new_n314, new_n315, new_n316, new_n317, new_n318, new_n319,
    new_n322, new_n323, new_n324, new_n325, new_n326, new_n327, new_n328,
    new_n330, new_n331, new_n332, new_n333, new_n334, new_n335, new_n336,
    new_n337, new_n338, new_n341, new_n342, new_n343, new_n345, new_n346,
    new_n347, new_n349, new_n350, new_n351, new_n353, new_n355, new_n356,
    new_n357, new_n359;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n03x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  orn002aa1n03x5               g002(.a(\a[2] ), .b(\b[1] ), .o(new_n98));
  nand02aa1n08x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  aob012aa1d18x5               g004(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(new_n100));
  nor002aa1d32x5               g005(.a(\b[2] ), .b(\a[3] ), .o1(new_n101));
  nand02aa1n06x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nanb02aa1d24x5               g007(.a(new_n101), .b(new_n102), .out0(new_n103));
  oab012aa1d30x5               g008(.a(new_n101), .b(\a[4] ), .c(\b[3] ), .out0(new_n104));
  aoai13aa1n06x5               g009(.a(new_n104), .b(new_n103), .c(new_n100), .d(new_n98), .o1(new_n105));
  nand42aa1n04x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  inv040aa1d32x5               g011(.a(\a[5] ), .o1(new_n107));
  inv040aa1d28x5               g012(.a(\b[4] ), .o1(new_n108));
  nand02aa1d20x5               g013(.a(new_n108), .b(new_n107), .o1(new_n109));
  oai112aa1n03x5               g014(.a(new_n109), .b(new_n106), .c(\b[5] ), .d(\a[6] ), .o1(new_n110));
  nanp02aa1n09x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  oai012aa1n02x5               g016(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  aoi022aa1d24x5               g018(.a(\b[6] ), .b(\a[7] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n114));
  oai112aa1n03x5               g019(.a(new_n114), .b(new_n113), .c(\b[6] ), .d(\a[7] ), .o1(new_n115));
  nor043aa1n03x5               g020(.a(new_n115), .b(new_n110), .c(new_n112), .o1(new_n116));
  oai022aa1d18x5               g021(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n117));
  nor042aa1n06x5               g022(.a(\b[6] ), .b(\a[7] ), .o1(new_n118));
  and002aa1n12x5               g023(.a(\b[6] ), .b(\a[7] ), .o(new_n119));
  aoi112aa1n06x5               g024(.a(new_n119), .b(new_n118), .c(\a[6] ), .d(\b[5] ), .o1(new_n120));
  xorc02aa1n12x5               g025(.a(\a[8] ), .b(\b[7] ), .out0(new_n121));
  nanp03aa1n02x5               g026(.a(new_n120), .b(new_n121), .c(new_n117), .o1(new_n122));
  inv000aa1d42x5               g027(.a(\a[8] ), .o1(new_n123));
  inv000aa1d42x5               g028(.a(\b[7] ), .o1(new_n124));
  tech160nm_fioaoi03aa1n05x5   g029(.a(new_n123), .b(new_n124), .c(new_n118), .o1(new_n125));
  nanp02aa1n03x5               g030(.a(new_n122), .b(new_n125), .o1(new_n126));
  tech160nm_fixnrc02aa1n02p5x5 g031(.a(\b[8] ), .b(\a[9] ), .out0(new_n127));
  inv000aa1n02x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n06x5               g033(.a(new_n128), .b(new_n126), .c(new_n105), .d(new_n116), .o1(new_n129));
  nor042aa1n06x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  nand02aa1n06x5               g035(.a(\b[9] ), .b(\a[10] ), .o1(new_n131));
  nanb02aa1n03x5               g036(.a(new_n130), .b(new_n131), .out0(new_n132));
  inv030aa1n03x5               g037(.a(new_n132), .o1(new_n133));
  xnbna2aa1n03x5               g038(.a(new_n133), .b(new_n129), .c(new_n97), .out0(\s[10] ));
  oaoi03aa1n02x5               g039(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n135));
  inv000aa1n02x5               g040(.a(new_n135), .o1(new_n136));
  aoai13aa1n04x5               g041(.a(new_n136), .b(new_n132), .c(new_n129), .d(new_n97), .o1(new_n137));
  xorb03aa1n02x5               g042(.a(new_n137), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g043(.a(\b[10] ), .b(\a[11] ), .o1(new_n139));
  nand02aa1d28x5               g044(.a(\b[10] ), .b(\a[11] ), .o1(new_n140));
  nor002aa1d32x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand02aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  norb02aa1n02x5               g047(.a(new_n142), .b(new_n141), .out0(new_n143));
  aoi112aa1n02x5               g048(.a(new_n139), .b(new_n143), .c(new_n137), .d(new_n140), .o1(new_n144));
  aoai13aa1n02x5               g049(.a(new_n143), .b(new_n139), .c(new_n137), .d(new_n140), .o1(new_n145));
  norb02aa1n02x7               g050(.a(new_n145), .b(new_n144), .out0(\s[12] ));
  nand42aa1n06x5               g051(.a(new_n116), .b(new_n105), .o1(new_n147));
  inv000aa1n02x5               g052(.a(new_n125), .o1(new_n148));
  aoi013aa1n09x5               g053(.a(new_n148), .b(new_n120), .c(new_n121), .d(new_n117), .o1(new_n149));
  nona23aa1d16x5               g054(.a(new_n142), .b(new_n140), .c(new_n139), .d(new_n141), .out0(new_n150));
  nanb03aa1n09x5               g055(.a(new_n150), .b(new_n128), .c(new_n133), .out0(new_n151));
  aoi012aa1n02x5               g056(.a(new_n151), .b(new_n147), .c(new_n149), .o1(new_n152));
  nanb03aa1n12x5               g057(.a(new_n141), .b(new_n142), .c(new_n140), .out0(new_n153));
  oai022aa1d24x5               g058(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n154));
  oai112aa1n06x5               g059(.a(new_n154), .b(new_n131), .c(\b[10] ), .d(\a[11] ), .o1(new_n155));
  aoi012aa1n12x5               g060(.a(new_n141), .b(new_n139), .c(new_n142), .o1(new_n156));
  oai012aa1d24x5               g061(.a(new_n156), .b(new_n155), .c(new_n153), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n06x5               g063(.a(new_n158), .b(new_n151), .c(new_n147), .d(new_n149), .o1(new_n159));
  nor002aa1d32x5               g064(.a(\b[12] ), .b(\a[13] ), .o1(new_n160));
  nand02aa1d06x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  norb02aa1n02x5               g066(.a(new_n161), .b(new_n160), .out0(new_n162));
  oaib12aa1n02x5               g067(.a(new_n156), .b(new_n160), .c(new_n161), .out0(new_n163));
  oab012aa1n02x4               g068(.a(new_n163), .b(new_n155), .c(new_n153), .out0(new_n164));
  aboi22aa1n03x5               g069(.a(new_n152), .b(new_n164), .c(new_n159), .d(new_n162), .out0(\s[13] ));
  inv000aa1d42x5               g070(.a(new_n160), .o1(new_n166));
  nanp02aa1n02x5               g071(.a(new_n159), .b(new_n162), .o1(new_n167));
  nor002aa1d32x5               g072(.a(\b[13] ), .b(\a[14] ), .o1(new_n168));
  nand02aa1n06x5               g073(.a(\b[13] ), .b(\a[14] ), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n169), .b(new_n168), .out0(new_n170));
  xnbna2aa1n03x5               g075(.a(new_n170), .b(new_n167), .c(new_n166), .out0(\s[14] ));
  nona23aa1d18x5               g076(.a(new_n169), .b(new_n161), .c(new_n160), .d(new_n168), .out0(new_n172));
  inv000aa1d42x5               g077(.a(new_n172), .o1(new_n173));
  oaoi03aa1n02x5               g078(.a(\a[14] ), .b(\b[13] ), .c(new_n166), .o1(new_n174));
  norp02aa1n04x5               g079(.a(\b[14] ), .b(\a[15] ), .o1(new_n175));
  nanp02aa1n04x5               g080(.a(\b[14] ), .b(\a[15] ), .o1(new_n176));
  norb02aa1n02x5               g081(.a(new_n176), .b(new_n175), .out0(new_n177));
  aoai13aa1n03x5               g082(.a(new_n177), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n178));
  aoi112aa1n02x5               g083(.a(new_n177), .b(new_n174), .c(new_n159), .d(new_n173), .o1(new_n179));
  norb02aa1n02x5               g084(.a(new_n178), .b(new_n179), .out0(\s[15] ));
  norp02aa1n09x5               g085(.a(\b[15] ), .b(\a[16] ), .o1(new_n181));
  nand22aa1n06x5               g086(.a(\b[15] ), .b(\a[16] ), .o1(new_n182));
  norb02aa1n02x5               g087(.a(new_n182), .b(new_n181), .out0(new_n183));
  norp02aa1n02x5               g088(.a(new_n183), .b(new_n175), .o1(new_n184));
  oai012aa1n02x5               g089(.a(new_n178), .b(\b[14] ), .c(\a[15] ), .o1(new_n185));
  aoi022aa1n02x5               g090(.a(new_n185), .b(new_n183), .c(new_n178), .d(new_n184), .o1(\s[16] ));
  aoi012aa1n03x5               g091(.a(new_n103), .b(new_n100), .c(new_n98), .o1(new_n187));
  inv000aa1d42x5               g092(.a(new_n104), .o1(new_n188));
  norb02aa1n02x5               g093(.a(new_n106), .b(new_n117), .out0(new_n189));
  oai012aa1n02x5               g094(.a(new_n113), .b(\b[6] ), .c(\a[7] ), .o1(new_n190));
  nona23aa1n03x5               g095(.a(new_n189), .b(new_n114), .c(new_n190), .d(new_n112), .out0(new_n191));
  oab012aa1n03x5               g096(.a(new_n191), .b(new_n187), .c(new_n188), .out0(new_n192));
  nona23aa1n06x5               g097(.a(new_n182), .b(new_n176), .c(new_n175), .d(new_n181), .out0(new_n193));
  nor042aa1n09x5               g098(.a(new_n193), .b(new_n172), .o1(new_n194));
  norb02aa1n06x5               g099(.a(new_n194), .b(new_n151), .out0(new_n195));
  tech160nm_fioai012aa1n05x5   g100(.a(new_n195), .b(new_n192), .c(new_n126), .o1(new_n196));
  nona32aa1n03x5               g101(.a(new_n194), .b(new_n150), .c(new_n132), .d(new_n127), .out0(new_n197));
  nanb03aa1n02x5               g102(.a(new_n181), .b(new_n182), .c(new_n176), .out0(new_n198));
  oai122aa1n02x7               g103(.a(new_n169), .b(new_n168), .c(new_n160), .d(\b[14] ), .e(\a[15] ), .o1(new_n199));
  aoi012aa1n02x5               g104(.a(new_n181), .b(new_n175), .c(new_n182), .o1(new_n200));
  oai012aa1n03x5               g105(.a(new_n200), .b(new_n199), .c(new_n198), .o1(new_n201));
  aoi012aa1n12x5               g106(.a(new_n201), .b(new_n157), .c(new_n194), .o1(new_n202));
  aoai13aa1n12x5               g107(.a(new_n202), .b(new_n197), .c(new_n147), .d(new_n149), .o1(new_n203));
  xorc02aa1n02x5               g108(.a(\a[17] ), .b(\b[16] ), .out0(new_n204));
  norp02aa1n02x5               g109(.a(new_n199), .b(new_n198), .o1(new_n205));
  nanb02aa1n02x5               g110(.a(new_n204), .b(new_n200), .out0(new_n206));
  aoi112aa1n02x5               g111(.a(new_n206), .b(new_n205), .c(new_n157), .d(new_n194), .o1(new_n207));
  aoi022aa1n02x5               g112(.a(new_n203), .b(new_n204), .c(new_n196), .d(new_n207), .o1(\s[17] ));
  inv040aa1n20x5               g113(.a(\a[17] ), .o1(new_n209));
  inv040aa1n16x5               g114(.a(\b[16] ), .o1(new_n210));
  nand42aa1n02x5               g115(.a(new_n210), .b(new_n209), .o1(new_n211));
  nanp02aa1n03x5               g116(.a(new_n203), .b(new_n204), .o1(new_n212));
  nor002aa1d32x5               g117(.a(\b[17] ), .b(\a[18] ), .o1(new_n213));
  nand02aa1n10x5               g118(.a(\b[17] ), .b(\a[18] ), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(new_n215));
  xnbna2aa1n03x5               g120(.a(new_n215), .b(new_n212), .c(new_n211), .out0(\s[18] ));
  nanp02aa1n02x5               g121(.a(\b[16] ), .b(\a[17] ), .o1(new_n217));
  nano32aa1n03x7               g122(.a(new_n213), .b(new_n211), .c(new_n214), .d(new_n217), .out0(new_n218));
  aoai13aa1n06x5               g123(.a(new_n214), .b(new_n213), .c(new_n209), .d(new_n210), .o1(new_n219));
  inv040aa1n02x5               g124(.a(new_n219), .o1(new_n220));
  xorc02aa1n12x5               g125(.a(\a[19] ), .b(\b[18] ), .out0(new_n221));
  aoai13aa1n06x5               g126(.a(new_n221), .b(new_n220), .c(new_n203), .d(new_n218), .o1(new_n222));
  aoi112aa1n02x5               g127(.a(new_n221), .b(new_n220), .c(new_n203), .d(new_n218), .o1(new_n223));
  norb02aa1n03x4               g128(.a(new_n222), .b(new_n223), .out0(\s[19] ));
  xnrc02aa1n02x5               g129(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  xorc02aa1n12x5               g130(.a(\a[20] ), .b(\b[19] ), .out0(new_n226));
  inv040aa1d32x5               g131(.a(\a[19] ), .o1(new_n227));
  inv000aa1d42x5               g132(.a(\b[18] ), .o1(new_n228));
  inv040aa1d32x5               g133(.a(\a[20] ), .o1(new_n229));
  inv040aa1d32x5               g134(.a(\b[19] ), .o1(new_n230));
  nanp02aa1n04x5               g135(.a(new_n230), .b(new_n229), .o1(new_n231));
  nanp02aa1n06x5               g136(.a(\b[19] ), .b(\a[20] ), .o1(new_n232));
  aoi022aa1n02x5               g137(.a(new_n231), .b(new_n232), .c(new_n228), .d(new_n227), .o1(new_n233));
  oaib12aa1n03x5               g138(.a(new_n222), .b(\b[18] ), .c(new_n227), .out0(new_n234));
  aoi022aa1n02x7               g139(.a(new_n234), .b(new_n226), .c(new_n222), .d(new_n233), .o1(\s[20] ));
  nand23aa1n04x5               g140(.a(new_n218), .b(new_n221), .c(new_n226), .o1(new_n236));
  inv040aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  nor042aa1n06x5               g142(.a(\b[18] ), .b(\a[19] ), .o1(new_n238));
  oai112aa1n04x5               g143(.a(new_n231), .b(new_n232), .c(new_n228), .d(new_n227), .o1(new_n239));
  tech160nm_fioaoi03aa1n03p5x5 g144(.a(new_n229), .b(new_n230), .c(new_n238), .o1(new_n240));
  oai013aa1n03x4               g145(.a(new_n240), .b(new_n239), .c(new_n219), .d(new_n238), .o1(new_n241));
  xorc02aa1n02x5               g146(.a(\a[21] ), .b(\b[20] ), .out0(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n241), .c(new_n203), .d(new_n237), .o1(new_n243));
  nona22aa1n09x5               g148(.a(new_n220), .b(new_n239), .c(new_n238), .out0(new_n244));
  nano22aa1n02x4               g149(.a(new_n242), .b(new_n244), .c(new_n240), .out0(new_n245));
  aobi12aa1n02x5               g150(.a(new_n245), .b(new_n203), .c(new_n237), .out0(new_n246));
  norb02aa1n03x4               g151(.a(new_n243), .b(new_n246), .out0(\s[21] ));
  xorc02aa1n02x5               g152(.a(\a[22] ), .b(\b[21] ), .out0(new_n248));
  nor022aa1n08x5               g153(.a(\b[20] ), .b(\a[21] ), .o1(new_n249));
  norp02aa1n02x5               g154(.a(new_n248), .b(new_n249), .o1(new_n250));
  inv000aa1d42x5               g155(.a(\a[21] ), .o1(new_n251));
  oaib12aa1n03x5               g156(.a(new_n243), .b(\b[20] ), .c(new_n251), .out0(new_n252));
  aoi022aa1n02x7               g157(.a(new_n252), .b(new_n248), .c(new_n243), .d(new_n250), .o1(\s[22] ));
  inv040aa1d32x5               g158(.a(\a[22] ), .o1(new_n254));
  xroi22aa1d06x4               g159(.a(new_n251), .b(\b[20] ), .c(new_n254), .d(\b[21] ), .out0(new_n255));
  inv030aa1n04x5               g160(.a(new_n255), .o1(new_n256));
  inv000aa1d42x5               g161(.a(\b[21] ), .o1(new_n257));
  oaoi03aa1n12x5               g162(.a(new_n254), .b(new_n257), .c(new_n249), .o1(new_n258));
  aoai13aa1n12x5               g163(.a(new_n258), .b(new_n256), .c(new_n244), .d(new_n240), .o1(new_n259));
  inv000aa1d42x5               g164(.a(new_n259), .o1(new_n260));
  nona22aa1n03x5               g165(.a(new_n203), .b(new_n236), .c(new_n256), .out0(new_n261));
  xorc02aa1n12x5               g166(.a(\a[23] ), .b(\b[22] ), .out0(new_n262));
  xnbna2aa1n03x5               g167(.a(new_n262), .b(new_n261), .c(new_n260), .out0(\s[23] ));
  aob012aa1n03x5               g168(.a(new_n262), .b(new_n261), .c(new_n260), .out0(new_n264));
  tech160nm_fixorc02aa1n02p5x5 g169(.a(\a[24] ), .b(\b[23] ), .out0(new_n265));
  norp02aa1n02x5               g170(.a(\b[22] ), .b(\a[23] ), .o1(new_n266));
  norp02aa1n02x5               g171(.a(new_n265), .b(new_n266), .o1(new_n267));
  aoi013aa1n03x5               g172(.a(new_n259), .b(new_n203), .c(new_n237), .d(new_n255), .o1(new_n268));
  oaoi03aa1n03x5               g173(.a(\a[23] ), .b(\b[22] ), .c(new_n268), .o1(new_n269));
  aoi022aa1n03x5               g174(.a(new_n269), .b(new_n265), .c(new_n264), .d(new_n267), .o1(\s[24] ));
  nanp02aa1n02x5               g175(.a(new_n265), .b(new_n262), .o1(new_n271));
  nona32aa1n06x5               g176(.a(new_n203), .b(new_n271), .c(new_n256), .d(new_n236), .out0(new_n272));
  and002aa1n02x5               g177(.a(new_n265), .b(new_n262), .o(new_n273));
  inv000aa1d42x5               g178(.a(\a[24] ), .o1(new_n274));
  inv000aa1d42x5               g179(.a(\b[23] ), .o1(new_n275));
  tech160nm_fioaoi03aa1n03p5x5 g180(.a(new_n274), .b(new_n275), .c(new_n266), .o1(new_n276));
  inv000aa1d42x5               g181(.a(new_n276), .o1(new_n277));
  tech160nm_fiaoi012aa1n04x5   g182(.a(new_n277), .b(new_n259), .c(new_n273), .o1(new_n278));
  xorc02aa1n12x5               g183(.a(\a[25] ), .b(\b[24] ), .out0(new_n279));
  aob012aa1n03x5               g184(.a(new_n279), .b(new_n272), .c(new_n278), .out0(new_n280));
  aoi112aa1n02x5               g185(.a(new_n279), .b(new_n277), .c(new_n259), .d(new_n273), .o1(new_n281));
  aobi12aa1n02x7               g186(.a(new_n280), .b(new_n281), .c(new_n272), .out0(\s[25] ));
  xorc02aa1n02x5               g187(.a(\a[26] ), .b(\b[25] ), .out0(new_n283));
  nor042aa1n03x5               g188(.a(\b[24] ), .b(\a[25] ), .o1(new_n284));
  norp02aa1n02x5               g189(.a(new_n283), .b(new_n284), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n284), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n279), .o1(new_n287));
  aoai13aa1n02x7               g192(.a(new_n286), .b(new_n287), .c(new_n272), .d(new_n278), .o1(new_n288));
  aoi022aa1n02x7               g193(.a(new_n288), .b(new_n283), .c(new_n280), .d(new_n285), .o1(\s[26] ));
  and002aa1n02x7               g194(.a(new_n283), .b(new_n279), .o(new_n290));
  aoai13aa1n09x5               g195(.a(new_n290), .b(new_n277), .c(new_n259), .d(new_n273), .o1(new_n291));
  nano23aa1n03x7               g196(.a(new_n236), .b(new_n271), .c(new_n290), .d(new_n255), .out0(new_n292));
  oao003aa1n12x5               g197(.a(\a[26] ), .b(\b[25] ), .c(new_n286), .carry(new_n293));
  inv000aa1d42x5               g198(.a(new_n293), .o1(new_n294));
  aoi012aa1n09x5               g199(.a(new_n294), .b(new_n203), .c(new_n292), .o1(new_n295));
  nanp02aa1n03x5               g200(.a(new_n295), .b(new_n291), .o1(new_n296));
  xorc02aa1n12x5               g201(.a(\a[27] ), .b(\b[26] ), .out0(new_n297));
  aoi112aa1n02x5               g202(.a(new_n297), .b(new_n294), .c(new_n203), .d(new_n292), .o1(new_n298));
  aoi022aa1n02x7               g203(.a(new_n296), .b(new_n297), .c(new_n291), .d(new_n298), .o1(\s[27] ));
  inv000aa1d42x5               g204(.a(new_n258), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n273), .b(new_n300), .c(new_n241), .d(new_n255), .o1(new_n301));
  aobi12aa1n12x5               g206(.a(new_n290), .b(new_n301), .c(new_n276), .out0(new_n302));
  nona23aa1n02x4               g207(.a(new_n237), .b(new_n290), .c(new_n271), .d(new_n256), .out0(new_n303));
  aoai13aa1n03x5               g208(.a(new_n293), .b(new_n303), .c(new_n196), .d(new_n202), .o1(new_n304));
  oai012aa1n02x5               g209(.a(new_n297), .b(new_n304), .c(new_n302), .o1(new_n305));
  tech160nm_fixorc02aa1n04x5   g210(.a(\a[28] ), .b(\b[27] ), .out0(new_n306));
  norp02aa1n02x5               g211(.a(\b[26] ), .b(\a[27] ), .o1(new_n307));
  norp02aa1n02x5               g212(.a(new_n306), .b(new_n307), .o1(new_n308));
  inv000aa1n03x5               g213(.a(new_n307), .o1(new_n309));
  inv000aa1d42x5               g214(.a(new_n297), .o1(new_n310));
  aoai13aa1n03x5               g215(.a(new_n309), .b(new_n310), .c(new_n295), .d(new_n291), .o1(new_n311));
  aoi022aa1n03x5               g216(.a(new_n311), .b(new_n306), .c(new_n305), .d(new_n308), .o1(\s[28] ));
  and002aa1n02x5               g217(.a(new_n306), .b(new_n297), .o(new_n313));
  oai012aa1n02x5               g218(.a(new_n313), .b(new_n304), .c(new_n302), .o1(new_n314));
  xorc02aa1n02x5               g219(.a(\a[29] ), .b(\b[28] ), .out0(new_n315));
  oao003aa1n02x5               g220(.a(\a[28] ), .b(\b[27] ), .c(new_n309), .carry(new_n316));
  norb02aa1n02x5               g221(.a(new_n316), .b(new_n315), .out0(new_n317));
  inv000aa1d42x5               g222(.a(new_n313), .o1(new_n318));
  aoai13aa1n06x5               g223(.a(new_n316), .b(new_n318), .c(new_n295), .d(new_n291), .o1(new_n319));
  aoi022aa1n03x5               g224(.a(new_n319), .b(new_n315), .c(new_n314), .d(new_n317), .o1(\s[29] ));
  xorb03aa1n02x5               g225(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g226(.a(new_n310), .b(new_n306), .c(new_n315), .out0(new_n322));
  oai012aa1n03x5               g227(.a(new_n322), .b(new_n304), .c(new_n302), .o1(new_n323));
  xorc02aa1n02x5               g228(.a(\a[30] ), .b(\b[29] ), .out0(new_n324));
  oao003aa1n02x5               g229(.a(\a[29] ), .b(\b[28] ), .c(new_n316), .carry(new_n325));
  norb02aa1n02x5               g230(.a(new_n325), .b(new_n324), .out0(new_n326));
  inv000aa1n02x5               g231(.a(new_n322), .o1(new_n327));
  aoai13aa1n03x5               g232(.a(new_n325), .b(new_n327), .c(new_n295), .d(new_n291), .o1(new_n328));
  aoi022aa1n03x5               g233(.a(new_n328), .b(new_n324), .c(new_n323), .d(new_n326), .o1(\s[30] ));
  nano32aa1n02x4               g234(.a(new_n310), .b(new_n324), .c(new_n306), .d(new_n315), .out0(new_n330));
  oai012aa1n03x5               g235(.a(new_n330), .b(new_n304), .c(new_n302), .o1(new_n331));
  xorc02aa1n02x5               g236(.a(\a[31] ), .b(\b[30] ), .out0(new_n332));
  and002aa1n02x5               g237(.a(\b[29] ), .b(\a[30] ), .o(new_n333));
  oabi12aa1n02x5               g238(.a(new_n332), .b(\a[30] ), .c(\b[29] ), .out0(new_n334));
  oab012aa1n02x4               g239(.a(new_n334), .b(new_n325), .c(new_n333), .out0(new_n335));
  inv000aa1n02x5               g240(.a(new_n330), .o1(new_n336));
  oao003aa1n02x5               g241(.a(\a[30] ), .b(\b[29] ), .c(new_n325), .carry(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n336), .c(new_n295), .d(new_n291), .o1(new_n338));
  aoi022aa1n03x5               g243(.a(new_n338), .b(new_n332), .c(new_n331), .d(new_n335), .o1(\s[31] ));
  xobna2aa1n03x5               g244(.a(new_n103), .b(new_n100), .c(new_n98), .out0(\s[3] ));
  xorc02aa1n02x5               g245(.a(\a[4] ), .b(\b[3] ), .out0(new_n341));
  norp03aa1n02x5               g246(.a(new_n187), .b(new_n341), .c(new_n101), .o1(new_n342));
  aboi22aa1n03x5               g247(.a(new_n187), .b(new_n104), .c(\a[4] ), .d(\b[3] ), .out0(new_n343));
  oaoi13aa1n02x5               g248(.a(new_n342), .b(new_n343), .c(\a[4] ), .d(\b[3] ), .o1(\s[4] ));
  inv000aa1d42x5               g249(.a(new_n109), .o1(new_n345));
  nano32aa1n03x7               g250(.a(new_n345), .b(new_n105), .c(new_n111), .d(new_n106), .out0(new_n346));
  aoi022aa1n02x5               g251(.a(new_n105), .b(new_n111), .c(new_n106), .d(new_n109), .o1(new_n347));
  norp02aa1n02x5               g252(.a(new_n346), .b(new_n347), .o1(\s[5] ));
  xorc02aa1n02x5               g253(.a(\a[6] ), .b(\b[5] ), .out0(new_n349));
  tech160nm_fioai012aa1n05x5   g254(.a(new_n349), .b(new_n346), .c(new_n345), .o1(new_n350));
  aoi113aa1n02x5               g255(.a(new_n349), .b(new_n345), .c(new_n105), .d(new_n111), .e(new_n106), .o1(new_n351));
  norb02aa1n02x5               g256(.a(new_n350), .b(new_n351), .out0(\s[6] ));
  tech160nm_fioai012aa1n03p5x5 g257(.a(new_n350), .b(\b[5] ), .c(\a[6] ), .o1(new_n353));
  xorb03aa1n02x5               g258(.a(new_n353), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  nona22aa1n03x5               g259(.a(new_n353), .b(new_n119), .c(new_n118), .out0(new_n355));
  tech160nm_fioai012aa1n03p5x5 g260(.a(new_n355), .b(\b[6] ), .c(\a[7] ), .o1(new_n356));
  norp02aa1n02x5               g261(.a(new_n121), .b(new_n118), .o1(new_n357));
  aoi022aa1n02x5               g262(.a(new_n356), .b(new_n121), .c(new_n355), .d(new_n357), .o1(\s[8] ));
  nano32aa1n02x4               g263(.a(new_n192), .b(new_n127), .c(new_n122), .d(new_n125), .out0(new_n359));
  norb02aa1n02x5               g264(.a(new_n129), .b(new_n359), .out0(\s[9] ));
endmodule


