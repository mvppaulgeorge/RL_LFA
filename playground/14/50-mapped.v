// Benchmark "adder" written by ABC on Wed Jul 17 19:36:14 2024

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
    new_n139, new_n140, new_n142, new_n143, new_n144, new_n145, new_n146,
    new_n147, new_n149, new_n150, new_n151, new_n152, new_n153, new_n154,
    new_n155, new_n156, new_n157, new_n158, new_n160, new_n161, new_n162,
    new_n163, new_n164, new_n165, new_n166, new_n167, new_n168, new_n169,
    new_n170, new_n171, new_n172, new_n173, new_n174, new_n176, new_n177,
    new_n178, new_n179, new_n180, new_n181, new_n183, new_n184, new_n185,
    new_n186, new_n187, new_n188, new_n189, new_n191, new_n192, new_n193,
    new_n194, new_n195, new_n196, new_n197, new_n198, new_n199, new_n201,
    new_n202, new_n203, new_n204, new_n205, new_n206, new_n207, new_n208,
    new_n209, new_n210, new_n211, new_n212, new_n213, new_n214, new_n215,
    new_n216, new_n218, new_n219, new_n220, new_n221, new_n222, new_n223,
    new_n225, new_n226, new_n227, new_n228, new_n229, new_n230, new_n233,
    new_n234, new_n235, new_n236, new_n237, new_n238, new_n239, new_n240,
    new_n241, new_n242, new_n244, new_n245, new_n246, new_n247, new_n248,
    new_n249, new_n250, new_n251, new_n252, new_n253, new_n254, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n264, new_n265, new_n266, new_n267, new_n268, new_n269, new_n270,
    new_n272, new_n273, new_n274, new_n275, new_n276, new_n277, new_n278,
    new_n279, new_n280, new_n281, new_n282, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n311, new_n312, new_n313, new_n314, new_n315, new_n316,
    new_n317, new_n319, new_n320, new_n321, new_n322, new_n323, new_n324,
    new_n325, new_n326, new_n327, new_n328, new_n330, new_n331, new_n332,
    new_n333, new_n334, new_n335, new_n337, new_n338, new_n339, new_n340,
    new_n341, new_n342, new_n343, new_n346, new_n347, new_n348, new_n349,
    new_n350, new_n351, new_n352, new_n353, new_n354, new_n355, new_n357,
    new_n358, new_n359, new_n360, new_n361, new_n362, new_n363, new_n364,
    new_n365, new_n366, new_n367, new_n368, new_n371, new_n372, new_n374,
    new_n376, new_n377, new_n378, new_n380, new_n381, new_n382, new_n383,
    new_n385, new_n387;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nor042aa1n06x5               g001(.a(\b[8] ), .b(\a[9] ), .o1(new_n97));
  inv000aa1n02x5               g002(.a(new_n97), .o1(new_n98));
  orn002aa1n02x7               g003(.a(\a[2] ), .b(\b[1] ), .o(new_n99));
  nanp02aa1n06x5               g004(.a(\b[0] ), .b(\a[1] ), .o1(new_n100));
  aob012aa1n12x5               g005(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(new_n101));
  nor022aa1n12x5               g006(.a(\b[2] ), .b(\a[3] ), .o1(new_n102));
  nand42aa1n06x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanb02aa1n02x5               g008(.a(new_n102), .b(new_n103), .out0(new_n104));
  norp02aa1n04x5               g009(.a(\b[3] ), .b(\a[4] ), .o1(new_n105));
  nand42aa1n20x5               g010(.a(\b[3] ), .b(\a[4] ), .o1(new_n106));
  norb03aa1n03x5               g011(.a(new_n106), .b(new_n102), .c(new_n105), .out0(new_n107));
  aoai13aa1n06x5               g012(.a(new_n107), .b(new_n104), .c(new_n99), .d(new_n101), .o1(new_n108));
  and002aa1n18x5               g013(.a(\b[7] ), .b(\a[8] ), .o(new_n109));
  nand42aa1n02x5               g014(.a(\b[4] ), .b(\a[5] ), .o1(new_n110));
  nand02aa1n04x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nano22aa1n03x7               g016(.a(new_n109), .b(new_n110), .c(new_n111), .out0(new_n112));
  tech160nm_fioai012aa1n05x5   g017(.a(new_n106), .b(\b[4] ), .c(\a[5] ), .o1(new_n113));
  nor042aa1n06x5               g018(.a(\b[5] ), .b(\a[6] ), .o1(new_n114));
  nand02aa1d28x5               g019(.a(\b[5] ), .b(\a[6] ), .o1(new_n115));
  nanb02aa1n12x5               g020(.a(new_n114), .b(new_n115), .out0(new_n116));
  oai022aa1d18x5               g021(.a(\a[7] ), .b(\b[6] ), .c(\b[7] ), .d(\a[8] ), .o1(new_n117));
  nor043aa1n02x5               g022(.a(new_n116), .b(new_n117), .c(new_n113), .o1(new_n118));
  nanp03aa1n06x5               g023(.a(new_n108), .b(new_n112), .c(new_n118), .o1(new_n119));
  inv000aa1n02x5               g024(.a(new_n109), .o1(new_n120));
  orn002aa1n24x5               g025(.a(\a[5] ), .b(\b[4] ), .o(new_n121));
  nanb03aa1n12x5               g026(.a(new_n114), .b(new_n121), .c(new_n115), .out0(new_n122));
  tech160nm_fioai012aa1n05x5   g027(.a(new_n115), .b(\b[6] ), .c(\a[7] ), .o1(new_n123));
  tech160nm_fioai012aa1n04x5   g028(.a(new_n111), .b(\b[7] ), .c(\a[8] ), .o1(new_n124));
  nor043aa1n03x5               g029(.a(new_n124), .b(new_n123), .c(new_n109), .o1(new_n125));
  aoi022aa1d24x5               g030(.a(new_n125), .b(new_n122), .c(new_n120), .d(new_n117), .o1(new_n126));
  xorc02aa1n12x5               g031(.a(\a[9] ), .b(\b[8] ), .out0(new_n127));
  inv000aa1d42x5               g032(.a(new_n127), .o1(new_n128));
  aoai13aa1n02x5               g033(.a(new_n98), .b(new_n128), .c(new_n119), .d(new_n126), .o1(new_n129));
  xorc02aa1n12x5               g034(.a(\a[10] ), .b(\b[9] ), .out0(new_n130));
  nand02aa1n02x5               g035(.a(new_n101), .b(new_n99), .o1(new_n131));
  norb02aa1n06x4               g036(.a(new_n103), .b(new_n102), .out0(new_n132));
  nona22aa1n02x4               g037(.a(new_n106), .b(new_n105), .c(new_n102), .out0(new_n133));
  tech160nm_fiaoi012aa1n05x5   g038(.a(new_n133), .b(new_n131), .c(new_n132), .o1(new_n134));
  inv000aa1n02x5               g039(.a(new_n113), .o1(new_n135));
  inv000aa1n02x5               g040(.a(new_n115), .o1(new_n136));
  nor003aa1n02x5               g041(.a(new_n117), .b(new_n136), .c(new_n114), .o1(new_n137));
  nand23aa1n03x5               g042(.a(new_n137), .b(new_n112), .c(new_n135), .o1(new_n138));
  oai012aa1d24x5               g043(.a(new_n126), .b(new_n134), .c(new_n138), .o1(new_n139));
  aoi112aa1n02x5               g044(.a(new_n130), .b(new_n97), .c(new_n139), .d(new_n127), .o1(new_n140));
  aoi012aa1n02x5               g045(.a(new_n140), .b(new_n129), .c(new_n130), .o1(\s[10] ));
  aoai13aa1n06x5               g046(.a(new_n130), .b(new_n97), .c(new_n139), .d(new_n127), .o1(new_n142));
  tech160nm_fioaoi03aa1n02p5x5 g047(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n143));
  inv000aa1d42x5               g048(.a(new_n143), .o1(new_n144));
  nor042aa1n02x5               g049(.a(\b[10] ), .b(\a[11] ), .o1(new_n145));
  nand02aa1n03x5               g050(.a(\b[10] ), .b(\a[11] ), .o1(new_n146));
  norb02aa1n02x7               g051(.a(new_n146), .b(new_n145), .out0(new_n147));
  xnbna2aa1n03x5               g052(.a(new_n147), .b(new_n142), .c(new_n144), .out0(\s[11] ));
  aoai13aa1n03x5               g053(.a(new_n147), .b(new_n143), .c(new_n129), .d(new_n130), .o1(new_n149));
  inv040aa1d32x5               g054(.a(\a[11] ), .o1(new_n150));
  inv000aa1d42x5               g055(.a(\b[10] ), .o1(new_n151));
  nand02aa1n08x5               g056(.a(new_n151), .b(new_n150), .o1(new_n152));
  inv000aa1d42x5               g057(.a(new_n147), .o1(new_n153));
  aoai13aa1n02x7               g058(.a(new_n152), .b(new_n153), .c(new_n142), .d(new_n144), .o1(new_n154));
  nor042aa1n04x5               g059(.a(\b[11] ), .b(\a[12] ), .o1(new_n155));
  nand02aa1n03x5               g060(.a(\b[11] ), .b(\a[12] ), .o1(new_n156));
  norb02aa1n02x5               g061(.a(new_n156), .b(new_n155), .out0(new_n157));
  aboi22aa1n03x5               g062(.a(new_n155), .b(new_n156), .c(new_n150), .d(new_n151), .out0(new_n158));
  aoi022aa1n02x7               g063(.a(new_n154), .b(new_n157), .c(new_n149), .d(new_n158), .o1(\s[12] ));
  nano23aa1n06x5               g064(.a(new_n145), .b(new_n155), .c(new_n156), .d(new_n146), .out0(new_n160));
  nand23aa1d12x5               g065(.a(new_n160), .b(new_n127), .c(new_n130), .o1(new_n161));
  inv000aa1d42x5               g066(.a(new_n161), .o1(new_n162));
  nanp02aa1n02x5               g067(.a(new_n139), .b(new_n162), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[9] ), .b(\a[10] ), .o1(new_n164));
  oai022aa1n02x5               g069(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n165));
  nanb03aa1n03x5               g070(.a(new_n155), .b(new_n156), .c(new_n146), .out0(new_n166));
  nano32aa1n03x7               g071(.a(new_n166), .b(new_n165), .c(new_n152), .d(new_n164), .out0(new_n167));
  tech160nm_fioaoi03aa1n03p5x5 g072(.a(\a[12] ), .b(\b[11] ), .c(new_n152), .o1(new_n168));
  norp02aa1n02x5               g073(.a(new_n167), .b(new_n168), .o1(new_n169));
  aoai13aa1n06x5               g074(.a(new_n169), .b(new_n161), .c(new_n119), .d(new_n126), .o1(new_n170));
  norp02aa1n12x5               g075(.a(\b[12] ), .b(\a[13] ), .o1(new_n171));
  tech160nm_finand02aa1n03p5x5 g076(.a(\b[12] ), .b(\a[13] ), .o1(new_n172));
  norb02aa1n02x5               g077(.a(new_n172), .b(new_n171), .out0(new_n173));
  norp03aa1n02x5               g078(.a(new_n167), .b(new_n168), .c(new_n173), .o1(new_n174));
  aoi022aa1n02x5               g079(.a(new_n170), .b(new_n173), .c(new_n163), .d(new_n174), .o1(\s[13] ));
  orn002aa1n02x5               g080(.a(\a[13] ), .b(\b[12] ), .o(new_n176));
  inv000aa1n02x5               g081(.a(new_n169), .o1(new_n177));
  aoai13aa1n03x5               g082(.a(new_n173), .b(new_n177), .c(new_n139), .d(new_n162), .o1(new_n178));
  nor022aa1n08x5               g083(.a(\b[13] ), .b(\a[14] ), .o1(new_n179));
  nand42aa1n03x5               g084(.a(\b[13] ), .b(\a[14] ), .o1(new_n180));
  norb02aa1n02x5               g085(.a(new_n180), .b(new_n179), .out0(new_n181));
  xnbna2aa1n03x5               g086(.a(new_n181), .b(new_n178), .c(new_n176), .out0(\s[14] ));
  nano23aa1n03x7               g087(.a(new_n171), .b(new_n179), .c(new_n180), .d(new_n172), .out0(new_n183));
  aoai13aa1n06x5               g088(.a(new_n183), .b(new_n177), .c(new_n139), .d(new_n162), .o1(new_n184));
  tech160nm_fioaoi03aa1n02p5x5 g089(.a(\a[14] ), .b(\b[13] ), .c(new_n176), .o1(new_n185));
  inv000aa1d42x5               g090(.a(new_n185), .o1(new_n186));
  nor042aa1d18x5               g091(.a(\b[14] ), .b(\a[15] ), .o1(new_n187));
  tech160nm_finand02aa1n05x5   g092(.a(\b[14] ), .b(\a[15] ), .o1(new_n188));
  norb02aa1n12x5               g093(.a(new_n188), .b(new_n187), .out0(new_n189));
  xnbna2aa1n03x5               g094(.a(new_n189), .b(new_n184), .c(new_n186), .out0(\s[15] ));
  aoai13aa1n02x5               g095(.a(new_n189), .b(new_n185), .c(new_n170), .d(new_n183), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n187), .o1(new_n192));
  inv000aa1d42x5               g097(.a(new_n189), .o1(new_n193));
  aoai13aa1n02x7               g098(.a(new_n192), .b(new_n193), .c(new_n184), .d(new_n186), .o1(new_n194));
  nor042aa1n12x5               g099(.a(\b[15] ), .b(\a[16] ), .o1(new_n195));
  nand02aa1d24x5               g100(.a(\b[15] ), .b(\a[16] ), .o1(new_n196));
  norb02aa1n09x5               g101(.a(new_n196), .b(new_n195), .out0(new_n197));
  inv000aa1d42x5               g102(.a(new_n195), .o1(new_n198));
  aoi012aa1n02x5               g103(.a(new_n187), .b(new_n198), .c(new_n196), .o1(new_n199));
  aoi022aa1n02x7               g104(.a(new_n194), .b(new_n197), .c(new_n191), .d(new_n199), .o1(\s[16] ));
  nano32aa1d12x5               g105(.a(new_n161), .b(new_n197), .c(new_n183), .d(new_n189), .out0(new_n201));
  nanp02aa1n02x5               g106(.a(new_n139), .b(new_n201), .o1(new_n202));
  nona23aa1n06x5               g107(.a(new_n180), .b(new_n172), .c(new_n171), .d(new_n179), .out0(new_n203));
  nano22aa1n12x5               g108(.a(new_n203), .b(new_n189), .c(new_n197), .out0(new_n204));
  nanb02aa1n12x5               g109(.a(new_n161), .b(new_n204), .out0(new_n205));
  inv000aa1d42x5               g110(.a(new_n196), .o1(new_n206));
  aoi022aa1d24x5               g111(.a(\b[14] ), .b(\a[15] ), .c(\a[14] ), .d(\b[13] ), .o1(new_n207));
  tech160nm_fioai012aa1n05x5   g112(.a(new_n207), .b(new_n179), .c(new_n171), .o1(new_n208));
  aoai13aa1n02x5               g113(.a(new_n198), .b(new_n206), .c(new_n208), .d(new_n192), .o1(new_n209));
  oaoi13aa1n09x5               g114(.a(new_n209), .b(new_n204), .c(new_n167), .d(new_n168), .o1(new_n210));
  aoai13aa1n12x5               g115(.a(new_n210), .b(new_n205), .c(new_n119), .d(new_n126), .o1(new_n211));
  xorc02aa1n02x5               g116(.a(\a[17] ), .b(\b[16] ), .out0(new_n212));
  tech160nm_fioai012aa1n04x5   g117(.a(new_n204), .b(new_n167), .c(new_n168), .o1(new_n213));
  nanp02aa1n03x5               g118(.a(new_n208), .b(new_n192), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n195), .c(new_n214), .d(new_n196), .o1(new_n215));
  and002aa1n02x5               g120(.a(new_n215), .b(new_n213), .o(new_n216));
  aoi022aa1n02x5               g121(.a(new_n211), .b(new_n212), .c(new_n202), .d(new_n216), .o1(\s[17] ));
  inv000aa1d42x5               g122(.a(\a[17] ), .o1(new_n218));
  nanb02aa1n02x5               g123(.a(\b[16] ), .b(new_n218), .out0(new_n219));
  aoi012aa1n02x5               g124(.a(new_n195), .b(new_n214), .c(new_n196), .o1(new_n220));
  nand22aa1n09x5               g125(.a(new_n213), .b(new_n220), .o1(new_n221));
  aoai13aa1n06x5               g126(.a(new_n212), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n222));
  xorc02aa1n02x5               g127(.a(\a[18] ), .b(\b[17] ), .out0(new_n223));
  xnbna2aa1n03x5               g128(.a(new_n223), .b(new_n222), .c(new_n219), .out0(\s[18] ));
  inv020aa1n04x5               g129(.a(\a[18] ), .o1(new_n225));
  xroi22aa1d06x4               g130(.a(new_n218), .b(\b[16] ), .c(new_n225), .d(\b[17] ), .out0(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n227));
  oaoi03aa1n02x5               g132(.a(\a[18] ), .b(\b[17] ), .c(new_n219), .o1(new_n228));
  inv000aa1d42x5               g133(.a(new_n228), .o1(new_n229));
  xorc02aa1n12x5               g134(.a(\a[19] ), .b(\b[18] ), .out0(new_n230));
  xnbna2aa1n03x5               g135(.a(new_n230), .b(new_n227), .c(new_n229), .out0(\s[19] ));
  xnrc02aa1n02x5               g136(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  aoai13aa1n03x5               g137(.a(new_n230), .b(new_n228), .c(new_n211), .d(new_n226), .o1(new_n233));
  inv000aa1d42x5               g138(.a(\a[19] ), .o1(new_n234));
  inv000aa1d42x5               g139(.a(\b[18] ), .o1(new_n235));
  nanp02aa1n02x5               g140(.a(new_n235), .b(new_n234), .o1(new_n236));
  inv000aa1d42x5               g141(.a(new_n230), .o1(new_n237));
  aoai13aa1n02x7               g142(.a(new_n236), .b(new_n237), .c(new_n227), .d(new_n229), .o1(new_n238));
  nor042aa1n04x5               g143(.a(\b[19] ), .b(\a[20] ), .o1(new_n239));
  nand22aa1n12x5               g144(.a(\b[19] ), .b(\a[20] ), .o1(new_n240));
  norb02aa1n06x4               g145(.a(new_n240), .b(new_n239), .out0(new_n241));
  aboi22aa1n03x5               g146(.a(new_n239), .b(new_n240), .c(new_n234), .d(new_n235), .out0(new_n242));
  aoi022aa1n02x7               g147(.a(new_n238), .b(new_n241), .c(new_n233), .d(new_n242), .o1(\s[20] ));
  nand23aa1n04x5               g148(.a(new_n226), .b(new_n230), .c(new_n241), .o1(new_n244));
  inv000aa1n02x5               g149(.a(new_n244), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n246));
  nor042aa1n04x5               g151(.a(\b[16] ), .b(\a[17] ), .o1(new_n247));
  oab012aa1n03x5               g152(.a(new_n247), .b(\a[18] ), .c(\b[17] ), .out0(new_n248));
  nanp02aa1n02x5               g153(.a(\b[18] ), .b(\a[19] ), .o1(new_n249));
  nano22aa1n03x5               g154(.a(new_n239), .b(new_n249), .c(new_n240), .out0(new_n250));
  nand42aa1n02x5               g155(.a(\b[17] ), .b(\a[18] ), .o1(new_n251));
  oai012aa1n03x5               g156(.a(new_n251), .b(\b[18] ), .c(\a[19] ), .o1(new_n252));
  nona22aa1n06x5               g157(.a(new_n250), .b(new_n248), .c(new_n252), .out0(new_n253));
  aoi013aa1n06x4               g158(.a(new_n239), .b(new_n240), .c(new_n234), .d(new_n235), .o1(new_n254));
  nanp02aa1n12x5               g159(.a(new_n253), .b(new_n254), .o1(new_n255));
  inv000aa1d42x5               g160(.a(new_n255), .o1(new_n256));
  aoai13aa1n02x5               g161(.a(new_n256), .b(new_n244), .c(new_n202), .d(new_n210), .o1(new_n257));
  nor002aa1d32x5               g162(.a(\b[20] ), .b(\a[21] ), .o1(new_n258));
  nand22aa1n02x5               g163(.a(\b[20] ), .b(\a[21] ), .o1(new_n259));
  norb02aa1n02x5               g164(.a(new_n259), .b(new_n258), .out0(new_n260));
  inv000aa1n02x5               g165(.a(new_n260), .o1(new_n261));
  and003aa1n02x5               g166(.a(new_n253), .b(new_n261), .c(new_n254), .o(new_n262));
  aoi022aa1n02x5               g167(.a(new_n257), .b(new_n260), .c(new_n246), .d(new_n262), .o1(\s[21] ));
  aoai13aa1n03x5               g168(.a(new_n260), .b(new_n255), .c(new_n211), .d(new_n245), .o1(new_n264));
  inv000aa1d42x5               g169(.a(new_n258), .o1(new_n265));
  aoai13aa1n04x5               g170(.a(new_n265), .b(new_n261), .c(new_n246), .d(new_n256), .o1(new_n266));
  nor042aa1n04x5               g171(.a(\b[21] ), .b(\a[22] ), .o1(new_n267));
  nand22aa1n03x5               g172(.a(\b[21] ), .b(\a[22] ), .o1(new_n268));
  norb02aa1n02x5               g173(.a(new_n268), .b(new_n267), .out0(new_n269));
  aoib12aa1n02x5               g174(.a(new_n258), .b(new_n268), .c(new_n267), .out0(new_n270));
  aoi022aa1n02x7               g175(.a(new_n266), .b(new_n269), .c(new_n264), .d(new_n270), .o1(\s[22] ));
  nona23aa1n02x4               g176(.a(new_n268), .b(new_n259), .c(new_n258), .d(new_n267), .out0(new_n272));
  nano32aa1n02x4               g177(.a(new_n272), .b(new_n226), .c(new_n230), .d(new_n241), .out0(new_n273));
  aoai13aa1n06x5               g178(.a(new_n273), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n274));
  nano23aa1n06x5               g179(.a(new_n258), .b(new_n267), .c(new_n268), .d(new_n259), .out0(new_n275));
  oaoi03aa1n02x5               g180(.a(\a[22] ), .b(\b[21] ), .c(new_n265), .o1(new_n276));
  tech160nm_fiaoi012aa1n04x5   g181(.a(new_n276), .b(new_n255), .c(new_n275), .o1(new_n277));
  nanp02aa1n02x5               g182(.a(new_n274), .b(new_n277), .o1(new_n278));
  nor002aa1n12x5               g183(.a(\b[22] ), .b(\a[23] ), .o1(new_n279));
  nanp02aa1n04x5               g184(.a(\b[22] ), .b(\a[23] ), .o1(new_n280));
  norb02aa1n09x5               g185(.a(new_n280), .b(new_n279), .out0(new_n281));
  aoi112aa1n02x5               g186(.a(new_n281), .b(new_n276), .c(new_n255), .d(new_n275), .o1(new_n282));
  aoi022aa1n02x5               g187(.a(new_n278), .b(new_n281), .c(new_n274), .d(new_n282), .o1(\s[23] ));
  inv000aa1n02x5               g188(.a(new_n277), .o1(new_n284));
  aoai13aa1n03x5               g189(.a(new_n281), .b(new_n284), .c(new_n211), .d(new_n273), .o1(new_n285));
  inv000aa1d42x5               g190(.a(new_n279), .o1(new_n286));
  inv000aa1d42x5               g191(.a(new_n281), .o1(new_n287));
  aoai13aa1n04x5               g192(.a(new_n286), .b(new_n287), .c(new_n274), .d(new_n277), .o1(new_n288));
  nor042aa1n04x5               g193(.a(\b[23] ), .b(\a[24] ), .o1(new_n289));
  nand02aa1n08x5               g194(.a(\b[23] ), .b(\a[24] ), .o1(new_n290));
  norb02aa1n02x7               g195(.a(new_n290), .b(new_n289), .out0(new_n291));
  inv000aa1d42x5               g196(.a(new_n289), .o1(new_n292));
  aoi012aa1n02x5               g197(.a(new_n279), .b(new_n292), .c(new_n290), .o1(new_n293));
  aoi022aa1n02x7               g198(.a(new_n288), .b(new_n291), .c(new_n285), .d(new_n293), .o1(\s[24] ));
  nano23aa1n02x4               g199(.a(new_n279), .b(new_n289), .c(new_n290), .d(new_n280), .out0(new_n295));
  nanp02aa1n03x5               g200(.a(new_n295), .b(new_n275), .o1(new_n296));
  nano32aa1n02x4               g201(.a(new_n296), .b(new_n226), .c(new_n230), .d(new_n241), .out0(new_n297));
  aoai13aa1n06x5               g202(.a(new_n297), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n298));
  nano22aa1n03x7               g203(.a(new_n272), .b(new_n281), .c(new_n291), .out0(new_n299));
  inv000aa1d42x5               g204(.a(new_n290), .o1(new_n300));
  aoai13aa1n03x5               g205(.a(new_n280), .b(new_n267), .c(new_n258), .d(new_n268), .o1(new_n301));
  aoai13aa1n04x5               g206(.a(new_n292), .b(new_n300), .c(new_n301), .d(new_n286), .o1(new_n302));
  aoi012aa1n02x7               g207(.a(new_n302), .b(new_n255), .c(new_n299), .o1(new_n303));
  nanp02aa1n02x5               g208(.a(new_n298), .b(new_n303), .o1(new_n304));
  xorc02aa1n12x5               g209(.a(\a[25] ), .b(\b[24] ), .out0(new_n305));
  tech160nm_fiaoi012aa1n04x5   g210(.a(new_n296), .b(new_n253), .c(new_n254), .o1(new_n306));
  aoi022aa1n02x5               g211(.a(new_n301), .b(new_n286), .c(\a[24] ), .d(\b[23] ), .o1(new_n307));
  inv000aa1d42x5               g212(.a(new_n305), .o1(new_n308));
  nano23aa1n02x4               g213(.a(new_n306), .b(new_n307), .c(new_n308), .d(new_n292), .out0(new_n309));
  aoi022aa1n02x5               g214(.a(new_n304), .b(new_n305), .c(new_n298), .d(new_n309), .o1(\s[25] ));
  inv000aa1n02x5               g215(.a(new_n303), .o1(new_n311));
  aoai13aa1n03x5               g216(.a(new_n305), .b(new_n311), .c(new_n211), .d(new_n297), .o1(new_n312));
  nor042aa1n03x5               g217(.a(\b[24] ), .b(\a[25] ), .o1(new_n313));
  inv000aa1d42x5               g218(.a(new_n313), .o1(new_n314));
  aoai13aa1n04x5               g219(.a(new_n314), .b(new_n308), .c(new_n298), .d(new_n303), .o1(new_n315));
  xorc02aa1n02x5               g220(.a(\a[26] ), .b(\b[25] ), .out0(new_n316));
  norp02aa1n02x5               g221(.a(new_n316), .b(new_n313), .o1(new_n317));
  aoi022aa1n02x7               g222(.a(new_n315), .b(new_n316), .c(new_n312), .d(new_n317), .o1(\s[26] ));
  and002aa1n02x5               g223(.a(new_n316), .b(new_n305), .o(new_n319));
  nano22aa1n06x5               g224(.a(new_n244), .b(new_n319), .c(new_n299), .out0(new_n320));
  aoai13aa1n04x5               g225(.a(new_n319), .b(new_n302), .c(new_n255), .d(new_n299), .o1(new_n321));
  oaoi03aa1n02x5               g226(.a(\a[26] ), .b(\b[25] ), .c(new_n314), .o1(new_n322));
  nanb02aa1n02x5               g227(.a(new_n322), .b(new_n321), .out0(new_n323));
  xorc02aa1n12x5               g228(.a(\a[27] ), .b(\b[26] ), .out0(new_n324));
  aoai13aa1n03x5               g229(.a(new_n324), .b(new_n323), .c(new_n211), .d(new_n320), .o1(new_n325));
  aoai13aa1n06x5               g230(.a(new_n320), .b(new_n221), .c(new_n139), .d(new_n201), .o1(new_n326));
  inv000aa1d42x5               g231(.a(new_n324), .o1(new_n327));
  nano32aa1n02x5               g232(.a(new_n322), .b(new_n326), .c(new_n327), .d(new_n321), .out0(new_n328));
  norb02aa1n02x7               g233(.a(new_n325), .b(new_n328), .out0(\s[27] ));
  oaoi13aa1n06x5               g234(.a(new_n322), .b(new_n319), .c(new_n306), .d(new_n302), .o1(new_n330));
  norp02aa1n02x5               g235(.a(\b[26] ), .b(\a[27] ), .o1(new_n331));
  inv000aa1n03x5               g236(.a(new_n331), .o1(new_n332));
  aoai13aa1n02x7               g237(.a(new_n332), .b(new_n327), .c(new_n326), .d(new_n330), .o1(new_n333));
  tech160nm_fixorc02aa1n03p5x5 g238(.a(\a[28] ), .b(\b[27] ), .out0(new_n334));
  norp02aa1n02x5               g239(.a(new_n334), .b(new_n331), .o1(new_n335));
  aoi022aa1n02x7               g240(.a(new_n333), .b(new_n334), .c(new_n325), .d(new_n335), .o1(\s[28] ));
  and002aa1n02x5               g241(.a(new_n334), .b(new_n324), .o(new_n337));
  aoai13aa1n03x5               g242(.a(new_n337), .b(new_n323), .c(new_n211), .d(new_n320), .o1(new_n338));
  inv000aa1d42x5               g243(.a(new_n337), .o1(new_n339));
  oao003aa1n02x5               g244(.a(\a[28] ), .b(\b[27] ), .c(new_n332), .carry(new_n340));
  aoai13aa1n02x7               g245(.a(new_n340), .b(new_n339), .c(new_n326), .d(new_n330), .o1(new_n341));
  tech160nm_fixorc02aa1n02p5x5 g246(.a(\a[29] ), .b(\b[28] ), .out0(new_n342));
  norb02aa1n02x5               g247(.a(new_n340), .b(new_n342), .out0(new_n343));
  aoi022aa1n02x7               g248(.a(new_n341), .b(new_n342), .c(new_n338), .d(new_n343), .o1(\s[29] ));
  xorb03aa1n02x5               g249(.a(new_n100), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n03x7               g250(.a(new_n327), .b(new_n334), .c(new_n342), .out0(new_n346));
  aoai13aa1n03x5               g251(.a(new_n346), .b(new_n323), .c(new_n211), .d(new_n320), .o1(new_n347));
  inv000aa1d42x5               g252(.a(new_n346), .o1(new_n348));
  inv000aa1d42x5               g253(.a(\b[28] ), .o1(new_n349));
  inv000aa1d42x5               g254(.a(\a[29] ), .o1(new_n350));
  oaib12aa1n02x5               g255(.a(new_n340), .b(\b[28] ), .c(new_n350), .out0(new_n351));
  oaib12aa1n02x5               g256(.a(new_n351), .b(new_n349), .c(\a[29] ), .out0(new_n352));
  aoai13aa1n02x7               g257(.a(new_n352), .b(new_n348), .c(new_n326), .d(new_n330), .o1(new_n353));
  xorc02aa1n02x5               g258(.a(\a[30] ), .b(\b[29] ), .out0(new_n354));
  oaoi13aa1n02x5               g259(.a(new_n354), .b(new_n351), .c(new_n350), .d(new_n349), .o1(new_n355));
  aoi022aa1n02x7               g260(.a(new_n353), .b(new_n354), .c(new_n347), .d(new_n355), .o1(\s[30] ));
  nanb02aa1n02x5               g261(.a(\b[30] ), .b(\a[31] ), .out0(new_n357));
  nanb02aa1n02x5               g262(.a(\a[31] ), .b(\b[30] ), .out0(new_n358));
  nanp02aa1n02x5               g263(.a(new_n358), .b(new_n357), .o1(new_n359));
  nano32aa1n06x5               g264(.a(new_n327), .b(new_n354), .c(new_n334), .d(new_n342), .out0(new_n360));
  aoai13aa1n03x5               g265(.a(new_n360), .b(new_n323), .c(new_n211), .d(new_n320), .o1(new_n361));
  inv000aa1d42x5               g266(.a(new_n360), .o1(new_n362));
  norp02aa1n02x5               g267(.a(\b[29] ), .b(\a[30] ), .o1(new_n363));
  aoi022aa1n02x5               g268(.a(\b[29] ), .b(\a[30] ), .c(\a[29] ), .d(\b[28] ), .o1(new_n364));
  aoi012aa1n02x5               g269(.a(new_n363), .b(new_n351), .c(new_n364), .o1(new_n365));
  aoai13aa1n02x7               g270(.a(new_n365), .b(new_n362), .c(new_n326), .d(new_n330), .o1(new_n366));
  oai112aa1n02x5               g271(.a(new_n357), .b(new_n358), .c(\b[29] ), .d(\a[30] ), .o1(new_n367));
  aoi012aa1n02x5               g272(.a(new_n367), .b(new_n351), .c(new_n364), .o1(new_n368));
  aoi022aa1n03x5               g273(.a(new_n366), .b(new_n359), .c(new_n361), .d(new_n368), .o1(\s[31] ));
  xnbna2aa1n03x5               g274(.a(new_n132), .b(new_n101), .c(new_n99), .out0(\s[3] ));
  norb02aa1n02x5               g275(.a(new_n106), .b(new_n105), .out0(new_n371));
  aoi012aa1n02x5               g276(.a(new_n102), .b(new_n131), .c(new_n132), .o1(new_n372));
  oai012aa1n02x5               g277(.a(new_n108), .b(new_n372), .c(new_n371), .o1(\s[4] ));
  aoi022aa1n02x5               g278(.a(new_n108), .b(new_n106), .c(new_n121), .d(new_n110), .o1(new_n374));
  aoi013aa1n02x4               g279(.a(new_n374), .b(new_n135), .c(new_n110), .d(new_n108), .o1(\s[5] ));
  inv000aa1d42x5               g280(.a(new_n116), .o1(new_n376));
  nanp03aa1n02x5               g281(.a(new_n108), .b(new_n110), .c(new_n135), .o1(new_n377));
  nanb02aa1n03x5               g282(.a(new_n122), .b(new_n377), .out0(new_n378));
  aoai13aa1n02x5               g283(.a(new_n378), .b(new_n376), .c(new_n121), .d(new_n377), .o1(\s[6] ));
  norp02aa1n02x5               g284(.a(\b[6] ), .b(\a[7] ), .o1(new_n380));
  inv000aa1d42x5               g285(.a(new_n380), .o1(new_n381));
  aoi022aa1n02x5               g286(.a(new_n378), .b(new_n115), .c(new_n111), .d(new_n381), .o1(new_n382));
  nanb03aa1n02x5               g287(.a(new_n123), .b(new_n378), .c(new_n111), .out0(new_n383));
  norb02aa1n02x5               g288(.a(new_n383), .b(new_n382), .out0(\s[7] ));
  xorc02aa1n02x5               g289(.a(\a[8] ), .b(\b[7] ), .out0(new_n385));
  xnbna2aa1n03x5               g290(.a(new_n385), .b(new_n383), .c(new_n381), .out0(\s[8] ));
  aoi122aa1n02x5               g291(.a(new_n127), .b(new_n120), .c(new_n117), .d(new_n125), .e(new_n122), .o1(new_n387));
  aoi022aa1n02x5               g292(.a(new_n139), .b(new_n127), .c(new_n387), .d(new_n119), .o1(\s[9] ));
endmodule


