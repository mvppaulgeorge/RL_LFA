// Benchmark "adder" written by ABC on Wed Jul 17 15:18:15 2024

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
    new_n118, new_n119, new_n120, new_n121, new_n122, new_n123, new_n125,
    new_n126, new_n127, new_n129, new_n130, new_n131, new_n132, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n154, new_n155, new_n157,
    new_n158, new_n159, new_n160, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n168, new_n169, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n212, new_n213, new_n214,
    new_n215, new_n216, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n227, new_n228, new_n229, new_n230,
    new_n231, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n250, new_n251, new_n252,
    new_n253, new_n254, new_n256, new_n257, new_n258, new_n259, new_n260,
    new_n261, new_n263, new_n264, new_n265, new_n266, new_n267, new_n268,
    new_n269, new_n270, new_n271, new_n272, new_n273, new_n274, new_n275,
    new_n277, new_n278, new_n279, new_n280, new_n281, new_n282, new_n284,
    new_n285, new_n286, new_n287, new_n288, new_n289, new_n290, new_n291,
    new_n294, new_n295, new_n296, new_n297, new_n298, new_n299, new_n300,
    new_n301, new_n303, new_n304, new_n305, new_n306, new_n307, new_n308,
    new_n309, new_n310, new_n313, new_n316, new_n318, new_n319, new_n321;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n06x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  norp02aa1n02x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nand22aa1n03x5               g003(.a(\b[0] ), .b(\a[1] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[1] ), .b(\a[2] ), .o1(new_n100));
  tech160nm_fiaoi012aa1n05x5   g005(.a(new_n98), .b(new_n99), .c(new_n100), .o1(new_n101));
  tech160nm_finand02aa1n03p5x5 g006(.a(\b[3] ), .b(\a[4] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[3] ), .b(\a[4] ), .o1(new_n103));
  nor022aa1n16x5               g008(.a(\b[2] ), .b(\a[3] ), .o1(new_n104));
  nand42aa1n03x5               g009(.a(\b[2] ), .b(\a[3] ), .o1(new_n105));
  nona23aa1n02x4               g010(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n106));
  tech160nm_fiaoi012aa1n03p5x5 g011(.a(new_n103), .b(new_n104), .c(new_n102), .o1(new_n107));
  oai012aa1n12x5               g012(.a(new_n107), .b(new_n106), .c(new_n101), .o1(new_n108));
  xorc02aa1n02x5               g013(.a(\a[5] ), .b(\b[4] ), .out0(new_n109));
  tech160nm_fixorc02aa1n02p5x5 g014(.a(\a[6] ), .b(\b[5] ), .out0(new_n110));
  tech160nm_finand02aa1n03p5x5 g015(.a(\b[7] ), .b(\a[8] ), .o1(new_n111));
  nor022aa1n08x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nor022aa1n04x5               g017(.a(\b[6] ), .b(\a[7] ), .o1(new_n113));
  nand42aa1n03x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nona23aa1n03x5               g019(.a(new_n111), .b(new_n114), .c(new_n113), .d(new_n112), .out0(new_n115));
  nano22aa1n03x7               g020(.a(new_n115), .b(new_n109), .c(new_n110), .out0(new_n116));
  oai012aa1n02x5               g021(.a(new_n111), .b(new_n113), .c(new_n112), .o1(new_n117));
  orn002aa1n24x5               g022(.a(\a[5] ), .b(\b[4] ), .o(new_n118));
  oaoi03aa1n09x5               g023(.a(\a[6] ), .b(\b[5] ), .c(new_n118), .o1(new_n119));
  oaib12aa1n06x5               g024(.a(new_n117), .b(new_n115), .c(new_n119), .out0(new_n120));
  tech160nm_fixorc02aa1n03p5x5 g025(.a(\a[9] ), .b(\b[8] ), .out0(new_n121));
  aoai13aa1n06x5               g026(.a(new_n121), .b(new_n120), .c(new_n116), .d(new_n108), .o1(new_n122));
  tech160nm_fixorc02aa1n03p5x5 g027(.a(\a[10] ), .b(\b[9] ), .out0(new_n123));
  xnbna2aa1n03x5               g028(.a(new_n123), .b(new_n122), .c(new_n97), .out0(\s[10] ));
  orn002aa1n02x5               g029(.a(\a[10] ), .b(\b[9] ), .o(new_n125));
  and002aa1n02x5               g030(.a(\b[9] ), .b(\a[10] ), .o(new_n126));
  aoi013aa1n02x4               g031(.a(new_n126), .b(new_n122), .c(new_n125), .d(new_n97), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor042aa1n06x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand42aa1d28x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  inv000aa1d42x5               g036(.a(new_n131), .o1(new_n132));
  aoi113aa1n03x7               g037(.a(new_n132), .b(new_n126), .c(new_n122), .d(new_n125), .e(new_n97), .o1(new_n133));
  nor042aa1n04x5               g038(.a(\b[11] ), .b(\a[12] ), .o1(new_n134));
  nand42aa1n16x5               g039(.a(\b[11] ), .b(\a[12] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(new_n136));
  norb03aa1n02x5               g041(.a(new_n136), .b(new_n133), .c(new_n129), .out0(new_n137));
  oab012aa1n03x5               g042(.a(new_n136), .b(new_n133), .c(new_n129), .out0(new_n138));
  norp02aa1n02x5               g043(.a(new_n138), .b(new_n137), .o1(\s[12] ));
  nano23aa1d15x5               g044(.a(new_n129), .b(new_n134), .c(new_n135), .d(new_n130), .out0(new_n140));
  inv000aa1d42x5               g045(.a(new_n140), .o1(new_n141));
  nano22aa1n02x4               g046(.a(new_n141), .b(new_n121), .c(new_n123), .out0(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n120), .c(new_n116), .d(new_n108), .o1(new_n143));
  oaoi03aa1n09x5               g048(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n144));
  aoi012aa1n02x5               g049(.a(new_n134), .b(new_n129), .c(new_n135), .o1(new_n145));
  aobi12aa1n09x5               g050(.a(new_n145), .b(new_n140), .c(new_n144), .out0(new_n146));
  nor002aa1d24x5               g051(.a(\b[12] ), .b(\a[13] ), .o1(new_n147));
  nand42aa1d28x5               g052(.a(\b[12] ), .b(\a[13] ), .o1(new_n148));
  norb02aa1n02x5               g053(.a(new_n148), .b(new_n147), .out0(new_n149));
  xnbna2aa1n03x5               g054(.a(new_n149), .b(new_n143), .c(new_n146), .out0(\s[13] ));
  inv000aa1d42x5               g055(.a(new_n147), .o1(new_n151));
  aob012aa1n02x5               g056(.a(new_n149), .b(new_n143), .c(new_n146), .out0(new_n152));
  nor042aa1n06x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nand42aa1d28x5               g058(.a(\b[13] ), .b(\a[14] ), .o1(new_n154));
  norb02aa1n02x5               g059(.a(new_n154), .b(new_n153), .out0(new_n155));
  xnbna2aa1n03x5               g060(.a(new_n155), .b(new_n152), .c(new_n151), .out0(\s[14] ));
  nano23aa1d15x5               g061(.a(new_n147), .b(new_n153), .c(new_n154), .d(new_n148), .out0(new_n157));
  inv000aa1d42x5               g062(.a(new_n157), .o1(new_n158));
  tech160nm_fioai012aa1n04x5   g063(.a(new_n154), .b(new_n153), .c(new_n147), .o1(new_n159));
  aoai13aa1n04x5               g064(.a(new_n159), .b(new_n158), .c(new_n143), .d(new_n146), .o1(new_n160));
  xorb03aa1n02x5               g065(.a(new_n160), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor042aa1n12x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nanp02aa1n03x5               g067(.a(\b[14] ), .b(\a[15] ), .o1(new_n163));
  nor042aa1n06x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  nand02aa1d06x5               g069(.a(\b[15] ), .b(\a[16] ), .o1(new_n165));
  nanb02aa1d24x5               g070(.a(new_n164), .b(new_n165), .out0(new_n166));
  inv000aa1d42x5               g071(.a(new_n166), .o1(new_n167));
  aoi112aa1n02x5               g072(.a(new_n162), .b(new_n167), .c(new_n160), .d(new_n163), .o1(new_n168));
  aoai13aa1n02x5               g073(.a(new_n167), .b(new_n162), .c(new_n160), .d(new_n163), .o1(new_n169));
  norb02aa1n02x7               g074(.a(new_n169), .b(new_n168), .out0(\s[16] ));
  nanb02aa1n03x5               g075(.a(new_n162), .b(new_n163), .out0(new_n171));
  nona22aa1d24x5               g076(.a(new_n157), .b(new_n171), .c(new_n166), .out0(new_n172));
  nano32aa1d15x5               g077(.a(new_n172), .b(new_n140), .c(new_n123), .d(new_n121), .out0(new_n173));
  aoai13aa1n12x5               g078(.a(new_n173), .b(new_n120), .c(new_n116), .d(new_n108), .o1(new_n174));
  nona23aa1n02x4               g079(.a(new_n165), .b(new_n163), .c(new_n162), .d(new_n164), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n162), .b(new_n165), .o1(new_n176));
  oai122aa1n03x5               g081(.a(new_n176), .b(new_n175), .c(new_n159), .d(\b[15] ), .e(\a[16] ), .o1(new_n177));
  oab012aa1n12x5               g082(.a(new_n177), .b(new_n146), .c(new_n172), .out0(new_n178));
  nanp02aa1n12x5               g083(.a(new_n174), .b(new_n178), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d06x4               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  oai022aa1d24x5               g091(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n187));
  oaib12aa1n18x5               g092(.a(new_n187), .b(new_n181), .c(\b[17] ), .out0(new_n188));
  inv040aa1n03x5               g093(.a(new_n188), .o1(new_n189));
  nor042aa1n06x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nanp02aa1n06x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n194));
  norb02aa1n02x7               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n06x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nand02aa1d08x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nona22aa1n02x5               g104(.a(new_n193), .b(new_n199), .c(new_n190), .out0(new_n200));
  inv000aa1d42x5               g105(.a(new_n199), .o1(new_n201));
  oaoi13aa1n06x5               g106(.a(new_n201), .b(new_n193), .c(\a[19] ), .d(\b[18] ), .o1(new_n202));
  norb02aa1n03x4               g107(.a(new_n200), .b(new_n202), .out0(\s[20] ));
  nano23aa1n09x5               g108(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n191), .out0(new_n204));
  nanp02aa1n02x5               g109(.a(new_n186), .b(new_n204), .o1(new_n205));
  nona23aa1n09x5               g110(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n206));
  aoi012aa1n06x5               g111(.a(new_n197), .b(new_n190), .c(new_n198), .o1(new_n207));
  oai012aa1n18x5               g112(.a(new_n207), .b(new_n206), .c(new_n188), .o1(new_n208));
  inv000aa1d42x5               g113(.a(new_n208), .o1(new_n209));
  aoai13aa1n04x5               g114(.a(new_n209), .b(new_n205), .c(new_n174), .d(new_n178), .o1(new_n210));
  xorb03aa1n02x5               g115(.a(new_n210), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n04x5               g116(.a(\b[20] ), .b(\a[21] ), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  xorc02aa1n02x5               g118(.a(\a[22] ), .b(\b[21] ), .out0(new_n214));
  aoi112aa1n02x5               g119(.a(new_n212), .b(new_n214), .c(new_n210), .d(new_n213), .o1(new_n215));
  aoai13aa1n03x5               g120(.a(new_n214), .b(new_n212), .c(new_n210), .d(new_n213), .o1(new_n216));
  norb02aa1n02x7               g121(.a(new_n216), .b(new_n215), .out0(\s[22] ));
  inv000aa1d42x5               g122(.a(\a[21] ), .o1(new_n218));
  inv000aa1d42x5               g123(.a(\a[22] ), .o1(new_n219));
  xroi22aa1d06x4               g124(.a(new_n218), .b(\b[20] ), .c(new_n219), .d(\b[21] ), .out0(new_n220));
  nanp03aa1n02x5               g125(.a(new_n220), .b(new_n186), .c(new_n204), .o1(new_n221));
  inv000aa1d42x5               g126(.a(\b[21] ), .o1(new_n222));
  oao003aa1n12x5               g127(.a(new_n219), .b(new_n222), .c(new_n212), .carry(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n208), .c(new_n220), .o1(new_n224));
  aoai13aa1n04x5               g129(.a(new_n224), .b(new_n221), .c(new_n174), .d(new_n178), .o1(new_n225));
  xorb03aa1n02x5               g130(.a(new_n225), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g131(.a(\b[22] ), .b(\a[23] ), .o1(new_n227));
  tech160nm_fixorc02aa1n05x5   g132(.a(\a[23] ), .b(\b[22] ), .out0(new_n228));
  xorc02aa1n03x5               g133(.a(\a[24] ), .b(\b[23] ), .out0(new_n229));
  aoi112aa1n02x5               g134(.a(new_n227), .b(new_n229), .c(new_n225), .d(new_n228), .o1(new_n230));
  aoai13aa1n03x5               g135(.a(new_n229), .b(new_n227), .c(new_n225), .d(new_n228), .o1(new_n231));
  norb02aa1n02x7               g136(.a(new_n231), .b(new_n230), .out0(\s[24] ));
  nano23aa1n02x4               g137(.a(new_n104), .b(new_n103), .c(new_n105), .d(new_n102), .out0(new_n233));
  nanb02aa1n02x5               g138(.a(new_n101), .b(new_n233), .out0(new_n234));
  nano23aa1n03x5               g139(.a(new_n113), .b(new_n112), .c(new_n114), .d(new_n111), .out0(new_n235));
  nanp03aa1n02x5               g140(.a(new_n235), .b(new_n109), .c(new_n110), .o1(new_n236));
  aobi12aa1n06x5               g141(.a(new_n117), .b(new_n235), .c(new_n119), .out0(new_n237));
  aoai13aa1n02x7               g142(.a(new_n237), .b(new_n236), .c(new_n234), .d(new_n107), .o1(new_n238));
  nor003aa1n02x5               g143(.a(new_n159), .b(new_n171), .c(new_n166), .o1(new_n239));
  aoi112aa1n02x7               g144(.a(new_n239), .b(new_n164), .c(new_n165), .d(new_n162), .o1(new_n240));
  oai012aa1n03x5               g145(.a(new_n240), .b(new_n146), .c(new_n172), .o1(new_n241));
  and002aa1n02x5               g146(.a(new_n229), .b(new_n228), .o(new_n242));
  inv000aa1n02x5               g147(.a(new_n242), .o1(new_n243));
  nano32aa1n02x4               g148(.a(new_n243), .b(new_n220), .c(new_n186), .d(new_n204), .out0(new_n244));
  aoai13aa1n02x5               g149(.a(new_n244), .b(new_n241), .c(new_n238), .d(new_n173), .o1(new_n245));
  inv020aa1n02x5               g150(.a(new_n207), .o1(new_n246));
  aoai13aa1n06x5               g151(.a(new_n220), .b(new_n246), .c(new_n204), .d(new_n189), .o1(new_n247));
  inv000aa1n02x5               g152(.a(new_n223), .o1(new_n248));
  aoi112aa1n02x5               g153(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n249));
  oab012aa1n02x4               g154(.a(new_n249), .b(\a[24] ), .c(\b[23] ), .out0(new_n250));
  aoai13aa1n12x5               g155(.a(new_n250), .b(new_n243), .c(new_n247), .d(new_n248), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  xnrc02aa1n12x5               g157(.a(\b[24] ), .b(\a[25] ), .out0(new_n253));
  inv000aa1d42x5               g158(.a(new_n253), .o1(new_n254));
  xnbna2aa1n03x5               g159(.a(new_n254), .b(new_n245), .c(new_n252), .out0(\s[25] ));
  nor042aa1n03x5               g160(.a(\b[24] ), .b(\a[25] ), .o1(new_n256));
  inv000aa1d42x5               g161(.a(new_n256), .o1(new_n257));
  aoai13aa1n03x5               g162(.a(new_n254), .b(new_n251), .c(new_n179), .d(new_n244), .o1(new_n258));
  tech160nm_fixnrc02aa1n04x5   g163(.a(\b[25] ), .b(\a[26] ), .out0(new_n259));
  nanp03aa1n03x5               g164(.a(new_n258), .b(new_n257), .c(new_n259), .o1(new_n260));
  tech160nm_fiaoi012aa1n02p5x5 g165(.a(new_n259), .b(new_n258), .c(new_n257), .o1(new_n261));
  norb02aa1n03x4               g166(.a(new_n260), .b(new_n261), .out0(\s[26] ));
  nor002aa1n06x5               g167(.a(new_n259), .b(new_n253), .o1(new_n263));
  nano22aa1n03x5               g168(.a(new_n221), .b(new_n242), .c(new_n263), .out0(new_n264));
  aoai13aa1n06x5               g169(.a(new_n264), .b(new_n241), .c(new_n238), .d(new_n173), .o1(new_n265));
  nanp02aa1n09x5               g170(.a(new_n251), .b(new_n263), .o1(new_n266));
  oao003aa1n02x5               g171(.a(\a[26] ), .b(\b[25] ), .c(new_n257), .carry(new_n267));
  xorc02aa1n12x5               g172(.a(\a[27] ), .b(\b[26] ), .out0(new_n268));
  inv000aa1d42x5               g173(.a(new_n268), .o1(new_n269));
  aoi013aa1n06x4               g174(.a(new_n269), .b(new_n265), .c(new_n266), .d(new_n267), .o1(new_n270));
  aobi12aa1n06x5               g175(.a(new_n264), .b(new_n174), .c(new_n178), .out0(new_n271));
  aoai13aa1n02x7               g176(.a(new_n242), .b(new_n223), .c(new_n208), .d(new_n220), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n263), .o1(new_n273));
  aoai13aa1n06x5               g178(.a(new_n267), .b(new_n273), .c(new_n272), .d(new_n250), .o1(new_n274));
  norp03aa1n02x5               g179(.a(new_n274), .b(new_n271), .c(new_n268), .o1(new_n275));
  norp02aa1n02x5               g180(.a(new_n270), .b(new_n275), .o1(\s[27] ));
  norp02aa1n02x5               g181(.a(\b[26] ), .b(\a[27] ), .o1(new_n277));
  inv040aa1n03x5               g182(.a(new_n277), .o1(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[27] ), .b(\a[28] ), .out0(new_n279));
  nano22aa1n03x5               g184(.a(new_n270), .b(new_n278), .c(new_n279), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n268), .b(new_n274), .c(new_n271), .o1(new_n281));
  tech160nm_fiaoi012aa1n02p5x5 g186(.a(new_n279), .b(new_n281), .c(new_n278), .o1(new_n282));
  norp02aa1n03x5               g187(.a(new_n282), .b(new_n280), .o1(\s[28] ));
  xnrc02aa1n02x5               g188(.a(\b[28] ), .b(\a[29] ), .out0(new_n284));
  norb02aa1n02x5               g189(.a(new_n268), .b(new_n279), .out0(new_n285));
  oaih12aa1n02x5               g190(.a(new_n285), .b(new_n274), .c(new_n271), .o1(new_n286));
  oao003aa1n02x5               g191(.a(\a[28] ), .b(\b[27] ), .c(new_n278), .carry(new_n287));
  tech160nm_fiaoi012aa1n02p5x5 g192(.a(new_n284), .b(new_n286), .c(new_n287), .o1(new_n288));
  inv000aa1n02x5               g193(.a(new_n285), .o1(new_n289));
  aoi013aa1n03x5               g194(.a(new_n289), .b(new_n265), .c(new_n266), .d(new_n267), .o1(new_n290));
  nano22aa1n02x4               g195(.a(new_n290), .b(new_n284), .c(new_n287), .out0(new_n291));
  norp02aa1n03x5               g196(.a(new_n288), .b(new_n291), .o1(\s[29] ));
  xorb03aa1n02x5               g197(.a(new_n99), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n06x5               g198(.a(new_n268), .b(new_n284), .c(new_n279), .out0(new_n294));
  oaih12aa1n02x5               g199(.a(new_n294), .b(new_n274), .c(new_n271), .o1(new_n295));
  oao003aa1n02x5               g200(.a(\a[29] ), .b(\b[28] ), .c(new_n287), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[29] ), .b(\a[30] ), .out0(new_n297));
  tech160nm_fiaoi012aa1n03p5x5 g202(.a(new_n297), .b(new_n295), .c(new_n296), .o1(new_n298));
  inv000aa1d42x5               g203(.a(new_n294), .o1(new_n299));
  aoi013aa1n02x4               g204(.a(new_n299), .b(new_n265), .c(new_n266), .d(new_n267), .o1(new_n300));
  nano22aa1n03x5               g205(.a(new_n300), .b(new_n296), .c(new_n297), .out0(new_n301));
  norp02aa1n03x5               g206(.a(new_n298), .b(new_n301), .o1(\s[30] ));
  xnrc02aa1n02x5               g207(.a(\b[30] ), .b(\a[31] ), .out0(new_n303));
  norb02aa1n02x5               g208(.a(new_n294), .b(new_n297), .out0(new_n304));
  inv000aa1n02x5               g209(.a(new_n304), .o1(new_n305));
  aoi013aa1n02x4               g210(.a(new_n305), .b(new_n265), .c(new_n266), .d(new_n267), .o1(new_n306));
  oao003aa1n02x5               g211(.a(\a[30] ), .b(\b[29] ), .c(new_n296), .carry(new_n307));
  nano22aa1n03x5               g212(.a(new_n306), .b(new_n303), .c(new_n307), .out0(new_n308));
  oaih12aa1n02x5               g213(.a(new_n304), .b(new_n274), .c(new_n271), .o1(new_n309));
  tech160nm_fiaoi012aa1n03p5x5 g214(.a(new_n303), .b(new_n309), .c(new_n307), .o1(new_n310));
  norp02aa1n03x5               g215(.a(new_n310), .b(new_n308), .o1(\s[31] ));
  xnrb03aa1n02x5               g216(.a(new_n101), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g217(.a(\a[3] ), .b(\b[2] ), .c(new_n101), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g219(.a(new_n108), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanp02aa1n02x5               g220(.a(new_n108), .b(new_n109), .o1(new_n316));
  xnbna2aa1n03x5               g221(.a(new_n110), .b(new_n316), .c(new_n118), .out0(\s[6] ));
  nanp03aa1n02x5               g222(.a(new_n316), .b(new_n118), .c(new_n110), .o1(new_n318));
  aob012aa1n02x5               g223(.a(new_n318), .b(\b[5] ), .c(\a[6] ), .out0(new_n319));
  xnrb03aa1n02x5               g224(.a(new_n319), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g225(.a(\a[7] ), .b(\b[6] ), .c(new_n319), .o1(new_n321));
  xorb03aa1n02x5               g226(.a(new_n321), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xorb03aa1n02x5               g227(.a(new_n238), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


