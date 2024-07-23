// Benchmark "adder" written by ABC on Wed Jul 17 18:05:50 2024

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
    new_n125, new_n126, new_n127, new_n128, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n136, new_n137, new_n138, new_n140,
    new_n141, new_n142, new_n143, new_n144, new_n145, new_n146, new_n147,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n156, new_n157, new_n158, new_n159, new_n161, new_n162, new_n163,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n170, new_n171,
    new_n172, new_n173, new_n174, new_n175, new_n176, new_n178, new_n179,
    new_n180, new_n181, new_n182, new_n183, new_n184, new_n186, new_n187,
    new_n188, new_n189, new_n190, new_n191, new_n192, new_n193, new_n194,
    new_n195, new_n197, new_n198, new_n199, new_n200, new_n202, new_n203,
    new_n204, new_n205, new_n206, new_n207, new_n208, new_n209, new_n212,
    new_n213, new_n214, new_n215, new_n216, new_n217, new_n219, new_n220,
    new_n221, new_n222, new_n223, new_n224, new_n225, new_n226, new_n227,
    new_n229, new_n230, new_n231, new_n232, new_n233, new_n235, new_n236,
    new_n237, new_n238, new_n239, new_n240, new_n241, new_n242, new_n243,
    new_n245, new_n246, new_n247, new_n248, new_n249, new_n251, new_n252,
    new_n253, new_n254, new_n255, new_n256, new_n257, new_n258, new_n259,
    new_n260, new_n261, new_n263, new_n264, new_n265, new_n266, new_n267,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n276, new_n278, new_n279, new_n280, new_n281, new_n282, new_n283,
    new_n284, new_n285, new_n286, new_n287, new_n288, new_n290, new_n291,
    new_n292, new_n293, new_n294, new_n295, new_n296, new_n299, new_n300,
    new_n301, new_n302, new_n303, new_n304, new_n305, new_n307, new_n308,
    new_n309, new_n310, new_n311, new_n312, new_n313, new_n316, new_n319,
    new_n321, new_n323;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv000aa1d42x5               g001(.a(\a[10] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\a[9] ), .o1(new_n98));
  inv000aa1d42x5               g003(.a(\b[8] ), .o1(new_n99));
  norp02aa1n24x5               g004(.a(\b[7] ), .b(\a[8] ), .o1(new_n100));
  nand42aa1n02x5               g005(.a(\b[7] ), .b(\a[8] ), .o1(new_n101));
  nor022aa1n16x5               g006(.a(\b[6] ), .b(\a[7] ), .o1(new_n102));
  nand42aa1n02x5               g007(.a(\b[6] ), .b(\a[7] ), .o1(new_n103));
  nona23aa1n03x5               g008(.a(new_n103), .b(new_n101), .c(new_n100), .d(new_n102), .out0(new_n104));
  tech160nm_fixnrc02aa1n04x5   g009(.a(\b[5] ), .b(\a[6] ), .out0(new_n105));
  inv000aa1d42x5               g010(.a(\a[5] ), .o1(new_n106));
  nanb02aa1n12x5               g011(.a(\b[4] ), .b(new_n106), .out0(new_n107));
  nanp02aa1n02x5               g012(.a(\b[4] ), .b(\a[5] ), .o1(new_n108));
  nano22aa1n02x4               g013(.a(new_n105), .b(new_n107), .c(new_n108), .out0(new_n109));
  and002aa1n02x5               g014(.a(\b[3] ), .b(\a[4] ), .o(new_n110));
  oa0022aa1n02x5               g015(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n111));
  inv040aa1d32x5               g016(.a(\a[3] ), .o1(new_n112));
  inv040aa1n08x5               g017(.a(\b[2] ), .o1(new_n113));
  nanp02aa1n04x5               g018(.a(new_n113), .b(new_n112), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nanp02aa1n02x5               g020(.a(new_n114), .b(new_n115), .o1(new_n116));
  nor042aa1n02x5               g021(.a(\b[1] ), .b(\a[2] ), .o1(new_n117));
  nanp02aa1n04x5               g022(.a(\b[1] ), .b(\a[2] ), .o1(new_n118));
  nand22aa1n09x5               g023(.a(\b[0] ), .b(\a[1] ), .o1(new_n119));
  aoi012aa1n12x5               g024(.a(new_n117), .b(new_n118), .c(new_n119), .o1(new_n120));
  oai012aa1n04x7               g025(.a(new_n111), .b(new_n120), .c(new_n116), .o1(new_n121));
  nona23aa1n03x5               g026(.a(new_n121), .b(new_n109), .c(new_n104), .d(new_n110), .out0(new_n122));
  nano23aa1n06x5               g027(.a(new_n100), .b(new_n102), .c(new_n103), .d(new_n101), .out0(new_n123));
  aoi112aa1n02x5               g028(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n124));
  oaoi03aa1n12x5               g029(.a(\a[6] ), .b(\b[5] ), .c(new_n107), .o1(new_n125));
  aoi112aa1n03x5               g030(.a(new_n124), .b(new_n100), .c(new_n123), .d(new_n125), .o1(new_n126));
  nand42aa1n04x5               g031(.a(new_n122), .b(new_n126), .o1(new_n127));
  oaoi03aa1n09x5               g032(.a(new_n98), .b(new_n99), .c(new_n127), .o1(new_n128));
  xorb03aa1n02x5               g033(.a(new_n128), .b(\b[9] ), .c(new_n97), .out0(\s[10] ));
  inv000aa1d42x5               g034(.a(\b[9] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(new_n130), .b(new_n97), .o1(new_n131));
  and002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o(new_n132));
  nor002aa1d32x5               g037(.a(\b[10] ), .b(\a[11] ), .o1(new_n133));
  nand42aa1d28x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  norb02aa1n02x5               g039(.a(new_n134), .b(new_n133), .out0(new_n135));
  inv000aa1d42x5               g040(.a(new_n135), .o1(new_n136));
  aoi112aa1n03x5               g041(.a(new_n136), .b(new_n132), .c(new_n128), .d(new_n131), .o1(new_n137));
  aoai13aa1n02x5               g042(.a(new_n136), .b(new_n132), .c(new_n128), .d(new_n131), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(\s[11] ));
  inv000aa1d42x5               g044(.a(new_n133), .o1(new_n140));
  nor002aa1d24x5               g045(.a(\b[11] ), .b(\a[12] ), .o1(new_n141));
  nand42aa1d28x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nanb02aa1n02x5               g047(.a(new_n141), .b(new_n142), .out0(new_n143));
  nano22aa1n02x4               g048(.a(new_n137), .b(new_n140), .c(new_n143), .out0(new_n144));
  nanp02aa1n03x5               g049(.a(new_n128), .b(new_n131), .o1(new_n145));
  nona22aa1n02x4               g050(.a(new_n145), .b(new_n136), .c(new_n132), .out0(new_n146));
  tech160nm_fiaoi012aa1n04x5   g051(.a(new_n143), .b(new_n146), .c(new_n140), .o1(new_n147));
  norp02aa1n02x5               g052(.a(new_n147), .b(new_n144), .o1(\s[12] ));
  xorc02aa1n12x5               g053(.a(\a[10] ), .b(\b[9] ), .out0(new_n149));
  xorc02aa1n12x5               g054(.a(\a[9] ), .b(\b[8] ), .out0(new_n150));
  nano23aa1d15x5               g055(.a(new_n133), .b(new_n141), .c(new_n142), .d(new_n134), .out0(new_n151));
  nand23aa1d12x5               g056(.a(new_n151), .b(new_n149), .c(new_n150), .o1(new_n152));
  nona23aa1n09x5               g057(.a(new_n142), .b(new_n134), .c(new_n133), .d(new_n141), .out0(new_n153));
  tech160nm_fioai012aa1n03p5x5 g058(.a(new_n142), .b(new_n141), .c(new_n133), .o1(new_n154));
  oai022aa1n04x7               g059(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n155));
  oaib12aa1n06x5               g060(.a(new_n155), .b(new_n130), .c(\a[10] ), .out0(new_n156));
  oai012aa1d24x5               g061(.a(new_n154), .b(new_n153), .c(new_n156), .o1(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n152), .c(new_n122), .d(new_n126), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n12x5               g065(.a(\b[12] ), .b(\a[13] ), .o1(new_n161));
  nand42aa1n06x5               g066(.a(\b[12] ), .b(\a[13] ), .o1(new_n162));
  aoi012aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n162), .o1(new_n163));
  xnrb03aa1n02x5               g068(.a(new_n163), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xorc02aa1n02x5               g069(.a(\a[5] ), .b(\b[4] ), .out0(new_n165));
  norb03aa1n03x5               g070(.a(new_n165), .b(new_n104), .c(new_n105), .out0(new_n166));
  oaoi13aa1n12x5               g071(.a(new_n110), .b(new_n111), .c(new_n120), .d(new_n116), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n123), .b(new_n125), .o1(new_n168));
  nona22aa1n02x4               g073(.a(new_n168), .b(new_n124), .c(new_n100), .out0(new_n169));
  inv000aa1d42x5               g074(.a(new_n152), .o1(new_n170));
  aoai13aa1n02x5               g075(.a(new_n170), .b(new_n169), .c(new_n167), .d(new_n166), .o1(new_n171));
  nor042aa1n06x5               g076(.a(\b[13] ), .b(\a[14] ), .o1(new_n172));
  nand42aa1n06x5               g077(.a(\b[13] ), .b(\a[14] ), .o1(new_n173));
  nona23aa1n03x5               g078(.a(new_n173), .b(new_n162), .c(new_n161), .d(new_n172), .out0(new_n174));
  oai012aa1n02x7               g079(.a(new_n173), .b(new_n172), .c(new_n161), .o1(new_n175));
  aoai13aa1n02x5               g080(.a(new_n175), .b(new_n174), .c(new_n171), .d(new_n158), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g082(.a(\b[14] ), .b(\a[15] ), .o1(new_n178));
  xnrc02aa1n12x5               g083(.a(\b[14] ), .b(\a[15] ), .out0(new_n179));
  inv040aa1n02x5               g084(.a(new_n179), .o1(new_n180));
  xnrc02aa1n12x5               g085(.a(\b[15] ), .b(\a[16] ), .out0(new_n181));
  inv040aa1n02x5               g086(.a(new_n181), .o1(new_n182));
  aoi112aa1n02x5               g087(.a(new_n182), .b(new_n178), .c(new_n176), .d(new_n180), .o1(new_n183));
  aoai13aa1n02x5               g088(.a(new_n182), .b(new_n178), .c(new_n176), .d(new_n180), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(\s[16] ));
  nano23aa1n03x7               g090(.a(new_n161), .b(new_n172), .c(new_n173), .d(new_n162), .out0(new_n186));
  nano32aa1d12x5               g091(.a(new_n152), .b(new_n182), .c(new_n180), .d(new_n186), .out0(new_n187));
  aoai13aa1n12x5               g092(.a(new_n187), .b(new_n169), .c(new_n167), .d(new_n166), .o1(new_n188));
  nor003aa1n03x5               g093(.a(new_n174), .b(new_n181), .c(new_n179), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n190));
  nor042aa1n02x5               g095(.a(\b[15] ), .b(\a[16] ), .o1(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  oai013aa1n03x5               g097(.a(new_n192), .b(new_n179), .c(new_n181), .d(new_n175), .o1(new_n193));
  aoi112aa1n09x5               g098(.a(new_n193), .b(new_n190), .c(new_n157), .d(new_n189), .o1(new_n194));
  nand02aa1d08x5               g099(.a(new_n188), .b(new_n194), .o1(new_n195));
  xorb03aa1n02x5               g100(.a(new_n195), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g101(.a(\a[18] ), .o1(new_n197));
  inv000aa1d42x5               g102(.a(\a[17] ), .o1(new_n198));
  inv000aa1d42x5               g103(.a(\b[16] ), .o1(new_n199));
  oaoi03aa1n02x5               g104(.a(new_n198), .b(new_n199), .c(new_n195), .o1(new_n200));
  xorb03aa1n02x5               g105(.a(new_n200), .b(\b[17] ), .c(new_n197), .out0(\s[18] ));
  xroi22aa1d04x5               g106(.a(new_n198), .b(\b[16] ), .c(new_n197), .d(\b[17] ), .out0(new_n202));
  nanp02aa1n02x5               g107(.a(new_n199), .b(new_n198), .o1(new_n203));
  oaoi03aa1n02x5               g108(.a(\a[18] ), .b(\b[17] ), .c(new_n203), .o1(new_n204));
  nor042aa1n04x5               g109(.a(\b[18] ), .b(\a[19] ), .o1(new_n205));
  nand02aa1n03x5               g110(.a(\b[18] ), .b(\a[19] ), .o1(new_n206));
  norb02aa1n02x5               g111(.a(new_n206), .b(new_n205), .out0(new_n207));
  aoai13aa1n06x5               g112(.a(new_n207), .b(new_n204), .c(new_n195), .d(new_n202), .o1(new_n208));
  aoi112aa1n02x5               g113(.a(new_n207), .b(new_n204), .c(new_n195), .d(new_n202), .o1(new_n209));
  norb02aa1n02x5               g114(.a(new_n208), .b(new_n209), .out0(\s[19] ));
  xnrc02aa1n02x5               g115(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g116(.a(\b[19] ), .b(\a[20] ), .o1(new_n212));
  nand42aa1n04x5               g117(.a(\b[19] ), .b(\a[20] ), .o1(new_n213));
  norb02aa1n02x5               g118(.a(new_n213), .b(new_n212), .out0(new_n214));
  nona22aa1n02x5               g119(.a(new_n208), .b(new_n214), .c(new_n205), .out0(new_n215));
  inv000aa1d42x5               g120(.a(new_n214), .o1(new_n216));
  oaoi13aa1n06x5               g121(.a(new_n216), .b(new_n208), .c(\a[19] ), .d(\b[18] ), .o1(new_n217));
  norb02aa1n03x4               g122(.a(new_n215), .b(new_n217), .out0(\s[20] ));
  nano23aa1n09x5               g123(.a(new_n205), .b(new_n212), .c(new_n213), .d(new_n206), .out0(new_n219));
  nanp02aa1n02x5               g124(.a(new_n202), .b(new_n219), .o1(new_n220));
  oai022aa1n02x7               g125(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n221));
  oaib12aa1n12x5               g126(.a(new_n221), .b(new_n197), .c(\b[17] ), .out0(new_n222));
  nona23aa1n09x5               g127(.a(new_n213), .b(new_n206), .c(new_n205), .d(new_n212), .out0(new_n223));
  aoi012aa1n12x5               g128(.a(new_n212), .b(new_n205), .c(new_n213), .o1(new_n224));
  oai012aa1d24x5               g129(.a(new_n224), .b(new_n223), .c(new_n222), .o1(new_n225));
  inv000aa1d42x5               g130(.a(new_n225), .o1(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n220), .c(new_n188), .d(new_n194), .o1(new_n227));
  xorb03aa1n02x5               g132(.a(new_n227), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor002aa1n02x5               g133(.a(\b[20] ), .b(\a[21] ), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[21] ), .b(\b[20] ), .out0(new_n230));
  xorc02aa1n02x5               g135(.a(\a[22] ), .b(\b[21] ), .out0(new_n231));
  aoi112aa1n02x5               g136(.a(new_n229), .b(new_n231), .c(new_n227), .d(new_n230), .o1(new_n232));
  aoai13aa1n02x5               g137(.a(new_n231), .b(new_n229), .c(new_n227), .d(new_n230), .o1(new_n233));
  norb02aa1n02x5               g138(.a(new_n233), .b(new_n232), .out0(\s[22] ));
  inv000aa1d42x5               g139(.a(\a[21] ), .o1(new_n235));
  inv000aa1d42x5               g140(.a(\a[22] ), .o1(new_n236));
  xroi22aa1d04x5               g141(.a(new_n235), .b(\b[20] ), .c(new_n236), .d(\b[21] ), .out0(new_n237));
  nanp03aa1n02x5               g142(.a(new_n237), .b(new_n202), .c(new_n219), .o1(new_n238));
  inv000aa1d42x5               g143(.a(\b[21] ), .o1(new_n239));
  oaoi03aa1n12x5               g144(.a(new_n236), .b(new_n239), .c(new_n229), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  aoi012aa1n02x5               g146(.a(new_n241), .b(new_n225), .c(new_n237), .o1(new_n242));
  aoai13aa1n06x5               g147(.a(new_n242), .b(new_n238), .c(new_n188), .d(new_n194), .o1(new_n243));
  xorb03aa1n02x5               g148(.a(new_n243), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g149(.a(\b[22] ), .b(\a[23] ), .o1(new_n245));
  tech160nm_fixorc02aa1n02p5x5 g150(.a(\a[23] ), .b(\b[22] ), .out0(new_n246));
  xorc02aa1n02x5               g151(.a(\a[24] ), .b(\b[23] ), .out0(new_n247));
  aoi112aa1n02x5               g152(.a(new_n245), .b(new_n247), .c(new_n243), .d(new_n246), .o1(new_n248));
  aoai13aa1n03x5               g153(.a(new_n247), .b(new_n245), .c(new_n243), .d(new_n246), .o1(new_n249));
  norb02aa1n02x5               g154(.a(new_n249), .b(new_n248), .out0(\s[24] ));
  and002aa1n12x5               g155(.a(new_n247), .b(new_n246), .o(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  nano32aa1n02x4               g157(.a(new_n252), .b(new_n237), .c(new_n202), .d(new_n219), .out0(new_n253));
  inv000aa1n02x5               g158(.a(new_n224), .o1(new_n254));
  aoai13aa1n03x5               g159(.a(new_n237), .b(new_n254), .c(new_n219), .d(new_n204), .o1(new_n255));
  orn002aa1n02x5               g160(.a(\a[23] ), .b(\b[22] ), .o(new_n256));
  oao003aa1n02x5               g161(.a(\a[24] ), .b(\b[23] ), .c(new_n256), .carry(new_n257));
  aoai13aa1n06x5               g162(.a(new_n257), .b(new_n252), .c(new_n255), .d(new_n240), .o1(new_n258));
  tech160nm_fixorc02aa1n05x5   g163(.a(\a[25] ), .b(\b[24] ), .out0(new_n259));
  aoai13aa1n06x5               g164(.a(new_n259), .b(new_n258), .c(new_n195), .d(new_n253), .o1(new_n260));
  aoi112aa1n02x5               g165(.a(new_n259), .b(new_n258), .c(new_n195), .d(new_n253), .o1(new_n261));
  norb02aa1n02x5               g166(.a(new_n260), .b(new_n261), .out0(\s[25] ));
  norp02aa1n02x5               g167(.a(\b[24] ), .b(\a[25] ), .o1(new_n263));
  tech160nm_fixorc02aa1n02p5x5 g168(.a(\a[26] ), .b(\b[25] ), .out0(new_n264));
  nona22aa1n06x5               g169(.a(new_n260), .b(new_n264), .c(new_n263), .out0(new_n265));
  inv000aa1n02x5               g170(.a(new_n263), .o1(new_n266));
  aobi12aa1n06x5               g171(.a(new_n264), .b(new_n260), .c(new_n266), .out0(new_n267));
  norb02aa1n03x4               g172(.a(new_n265), .b(new_n267), .out0(\s[26] ));
  nanp02aa1n02x5               g173(.a(new_n157), .b(new_n189), .o1(new_n269));
  nona22aa1n02x4               g174(.a(new_n269), .b(new_n193), .c(new_n190), .out0(new_n270));
  and002aa1n06x5               g175(.a(new_n264), .b(new_n259), .o(new_n271));
  nano22aa1n03x7               g176(.a(new_n238), .b(new_n251), .c(new_n271), .out0(new_n272));
  aoai13aa1n06x5               g177(.a(new_n272), .b(new_n270), .c(new_n127), .d(new_n187), .o1(new_n273));
  oao003aa1n02x5               g178(.a(\a[26] ), .b(\b[25] ), .c(new_n266), .carry(new_n274));
  aobi12aa1n09x5               g179(.a(new_n274), .b(new_n258), .c(new_n271), .out0(new_n275));
  xorc02aa1n12x5               g180(.a(\a[27] ), .b(\b[26] ), .out0(new_n276));
  xnbna2aa1n03x5               g181(.a(new_n276), .b(new_n275), .c(new_n273), .out0(\s[27] ));
  norp02aa1n02x5               g182(.a(\b[26] ), .b(\a[27] ), .o1(new_n278));
  inv040aa1n03x5               g183(.a(new_n278), .o1(new_n279));
  aobi12aa1n02x7               g184(.a(new_n276), .b(new_n275), .c(new_n273), .out0(new_n280));
  xnrc02aa1n02x5               g185(.a(\b[27] ), .b(\a[28] ), .out0(new_n281));
  nano22aa1n02x4               g186(.a(new_n280), .b(new_n279), .c(new_n281), .out0(new_n282));
  aobi12aa1n06x5               g187(.a(new_n272), .b(new_n188), .c(new_n194), .out0(new_n283));
  aoai13aa1n06x5               g188(.a(new_n251), .b(new_n241), .c(new_n225), .d(new_n237), .o1(new_n284));
  inv000aa1d42x5               g189(.a(new_n271), .o1(new_n285));
  aoai13aa1n06x5               g190(.a(new_n274), .b(new_n285), .c(new_n284), .d(new_n257), .o1(new_n286));
  tech160nm_fioai012aa1n05x5   g191(.a(new_n276), .b(new_n286), .c(new_n283), .o1(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n281), .b(new_n287), .c(new_n279), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n282), .o1(\s[28] ));
  norb02aa1n02x5               g194(.a(new_n276), .b(new_n281), .out0(new_n290));
  aobi12aa1n02x7               g195(.a(new_n290), .b(new_n275), .c(new_n273), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[28] ), .b(\b[27] ), .c(new_n279), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[28] ), .b(\a[29] ), .out0(new_n293));
  nano22aa1n03x5               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n290), .b(new_n286), .c(new_n283), .o1(new_n295));
  aoi012aa1n03x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n03x5               g201(.a(new_n296), .b(new_n294), .o1(\s[29] ));
  xorb03aa1n02x5               g202(.a(new_n119), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g203(.a(new_n276), .b(new_n293), .c(new_n281), .out0(new_n299));
  aobi12aa1n02x7               g204(.a(new_n299), .b(new_n275), .c(new_n273), .out0(new_n300));
  oao003aa1n02x5               g205(.a(\a[29] ), .b(\b[28] ), .c(new_n292), .carry(new_n301));
  xnrc02aa1n02x5               g206(.a(\b[29] ), .b(\a[30] ), .out0(new_n302));
  nano22aa1n03x5               g207(.a(new_n300), .b(new_n301), .c(new_n302), .out0(new_n303));
  oaih12aa1n02x5               g208(.a(new_n299), .b(new_n286), .c(new_n283), .o1(new_n304));
  tech160nm_fiaoi012aa1n02p5x5 g209(.a(new_n302), .b(new_n304), .c(new_n301), .o1(new_n305));
  norp02aa1n03x5               g210(.a(new_n305), .b(new_n303), .o1(\s[30] ));
  norb02aa1n02x5               g211(.a(new_n299), .b(new_n302), .out0(new_n307));
  aobi12aa1n02x7               g212(.a(new_n307), .b(new_n275), .c(new_n273), .out0(new_n308));
  oao003aa1n02x5               g213(.a(\a[30] ), .b(\b[29] ), .c(new_n301), .carry(new_n309));
  xnrc02aa1n02x5               g214(.a(\b[30] ), .b(\a[31] ), .out0(new_n310));
  nano22aa1n03x5               g215(.a(new_n308), .b(new_n309), .c(new_n310), .out0(new_n311));
  oaih12aa1n02x5               g216(.a(new_n307), .b(new_n286), .c(new_n283), .o1(new_n312));
  aoi012aa1n03x5               g217(.a(new_n310), .b(new_n312), .c(new_n309), .o1(new_n313));
  norp02aa1n03x5               g218(.a(new_n313), .b(new_n311), .o1(\s[31] ));
  xnbna2aa1n03x5               g219(.a(new_n120), .b(new_n114), .c(new_n115), .out0(\s[3] ));
  oaoi03aa1n02x5               g220(.a(\a[3] ), .b(\b[2] ), .c(new_n120), .o1(new_n316));
  xorb03aa1n02x5               g221(.a(new_n316), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g222(.a(new_n167), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb03aa1n02x5               g223(.a(new_n110), .b(new_n121), .c(new_n165), .out0(new_n319));
  xobna2aa1n03x5               g224(.a(new_n105), .b(new_n319), .c(new_n107), .out0(\s[6] ));
  tech160nm_fiao0012aa1n02p5x5 g225(.a(new_n125), .b(new_n167), .c(new_n109), .o(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  aoi012aa1n02x5               g227(.a(new_n102), .b(new_n321), .c(new_n103), .o1(new_n323));
  xnrb03aa1n02x5               g228(.a(new_n323), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g229(.a(new_n150), .b(new_n122), .c(new_n126), .out0(\s[9] ));
endmodule


