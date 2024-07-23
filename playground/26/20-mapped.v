// Benchmark "adder" written by ABC on Thu Jul 18 01:26:54 2024

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
    new_n125, new_n127, new_n128, new_n129, new_n130, new_n131, new_n132,
    new_n134, new_n136, new_n137, new_n138, new_n139, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n149,
    new_n150, new_n152, new_n153, new_n154, new_n155, new_n156, new_n157,
    new_n159, new_n160, new_n161, new_n162, new_n163, new_n165, new_n166,
    new_n167, new_n168, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n175, new_n176, new_n177, new_n178, new_n180, new_n181, new_n182,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n190, new_n191,
    new_n192, new_n193, new_n194, new_n195, new_n197, new_n198, new_n199,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n207,
    new_n208, new_n209, new_n210, new_n211, new_n213, new_n214, new_n215,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n223,
    new_n224, new_n225, new_n226, new_n227, new_n229, new_n230, new_n231,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n241, new_n242, new_n243, new_n244, new_n245, new_n247,
    new_n248, new_n249, new_n250, new_n251, new_n252, new_n253, new_n255,
    new_n256, new_n257, new_n258, new_n259, new_n260, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n267, new_n268, new_n269, new_n270,
    new_n271, new_n272, new_n273, new_n276, new_n277, new_n278, new_n279,
    new_n280, new_n281, new_n282, new_n284, new_n285, new_n286, new_n287,
    new_n288, new_n289, new_n290, new_n293, new_n296, new_n297, new_n299,
    new_n300, new_n301, new_n302, new_n304, new_n305, new_n306;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n24x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor002aa1d32x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand02aa1n06x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norp02aa1n12x5               g004(.a(\b[6] ), .b(\a[7] ), .o1(new_n100));
  nanp02aa1n04x5               g005(.a(\b[6] ), .b(\a[7] ), .o1(new_n101));
  nona23aa1n09x5               g006(.a(new_n101), .b(new_n99), .c(new_n98), .d(new_n100), .out0(new_n102));
  xnrc02aa1n02x5               g007(.a(\b[5] ), .b(\a[6] ), .out0(new_n103));
  xnrc02aa1n02x5               g008(.a(\b[4] ), .b(\a[5] ), .out0(new_n104));
  norp03aa1d12x5               g009(.a(new_n102), .b(new_n103), .c(new_n104), .o1(new_n105));
  norp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  tech160nm_fiaoi012aa1n05x5   g013(.a(new_n106), .b(new_n107), .c(new_n108), .o1(new_n109));
  tech160nm_finor002aa1n03p5x5 g014(.a(\b[3] ), .b(\a[4] ), .o1(new_n110));
  nand02aa1d04x5               g015(.a(\b[3] ), .b(\a[4] ), .o1(new_n111));
  nor022aa1n16x5               g016(.a(\b[2] ), .b(\a[3] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[2] ), .b(\a[3] ), .o1(new_n113));
  nona23aa1n03x5               g018(.a(new_n113), .b(new_n111), .c(new_n110), .d(new_n112), .out0(new_n114));
  ao0012aa1n03x5               g019(.a(new_n110), .b(new_n112), .c(new_n111), .o(new_n115));
  oabi12aa1n09x5               g020(.a(new_n115), .b(new_n114), .c(new_n109), .out0(new_n116));
  inv000aa1d42x5               g021(.a(new_n98), .o1(new_n117));
  nanp02aa1n02x5               g022(.a(new_n100), .b(new_n99), .o1(new_n118));
  inv000aa1d42x5               g023(.a(\b[5] ), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  oaib12aa1n02x5               g025(.a(new_n120), .b(new_n119), .c(\a[6] ), .out0(new_n121));
  oai112aa1n06x5               g026(.a(new_n117), .b(new_n118), .c(new_n102), .d(new_n121), .o1(new_n122));
  xorc02aa1n02x5               g027(.a(\a[9] ), .b(\b[8] ), .out0(new_n123));
  aoai13aa1n02x5               g028(.a(new_n123), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n124));
  xnrc02aa1n12x5               g029(.a(\b[9] ), .b(\a[10] ), .out0(new_n125));
  xobna2aa1n03x5               g030(.a(new_n125), .b(new_n124), .c(new_n97), .out0(\s[10] ));
  nand42aa1n02x5               g031(.a(new_n116), .b(new_n105), .o1(new_n127));
  nanb02aa1n06x5               g032(.a(new_n122), .b(new_n127), .out0(new_n128));
  tech160nm_fioaoi03aa1n03p5x5 g033(.a(\a[10] ), .b(\b[9] ), .c(new_n97), .o1(new_n129));
  nanp02aa1n02x5               g034(.a(\b[8] ), .b(\a[9] ), .o1(new_n130));
  nano22aa1n02x5               g035(.a(new_n125), .b(new_n97), .c(new_n130), .out0(new_n131));
  aoi012aa1n03x5               g036(.a(new_n129), .b(new_n128), .c(new_n131), .o1(new_n132));
  xnrb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n03x5               g038(.a(\a[11] ), .b(\b[10] ), .c(new_n132), .o1(new_n134));
  xorb03aa1n02x5               g039(.a(new_n134), .b(\b[11] ), .c(\a[12] ), .out0(\s[12] ));
  nor002aa1n03x5               g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand42aa1n02x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  norp02aa1n06x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  nanp02aa1n04x5               g043(.a(\b[11] ), .b(\a[12] ), .o1(new_n139));
  nano23aa1n09x5               g044(.a(new_n136), .b(new_n138), .c(new_n139), .d(new_n137), .out0(new_n140));
  nano22aa1n02x4               g045(.a(new_n125), .b(new_n140), .c(new_n123), .out0(new_n141));
  aoai13aa1n06x5               g046(.a(new_n141), .b(new_n122), .c(new_n116), .d(new_n105), .o1(new_n142));
  tech160nm_fiao0012aa1n02p5x5 g047(.a(new_n138), .b(new_n136), .c(new_n139), .o(new_n143));
  tech160nm_fiaoi012aa1n04x5   g048(.a(new_n143), .b(new_n140), .c(new_n129), .o1(new_n144));
  nor002aa1n20x5               g049(.a(\b[12] ), .b(\a[13] ), .o1(new_n145));
  nand42aa1d28x5               g050(.a(\b[12] ), .b(\a[13] ), .o1(new_n146));
  nanb02aa1n02x5               g051(.a(new_n145), .b(new_n146), .out0(new_n147));
  xobna2aa1n03x5               g052(.a(new_n147), .b(new_n142), .c(new_n144), .out0(\s[13] ));
  inv000aa1d42x5               g053(.a(new_n145), .o1(new_n149));
  aoai13aa1n02x5               g054(.a(new_n149), .b(new_n147), .c(new_n142), .d(new_n144), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor042aa1n04x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nand42aa1n16x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  nano23aa1d15x5               g058(.a(new_n145), .b(new_n152), .c(new_n153), .d(new_n146), .out0(new_n154));
  inv000aa1d42x5               g059(.a(new_n154), .o1(new_n155));
  oai012aa1n02x5               g060(.a(new_n153), .b(new_n152), .c(new_n145), .o1(new_n156));
  aoai13aa1n03x5               g061(.a(new_n156), .b(new_n155), .c(new_n142), .d(new_n144), .o1(new_n157));
  xorb03aa1n02x5               g062(.a(new_n157), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g063(.a(\b[14] ), .b(\a[15] ), .o1(new_n159));
  xorc02aa1n12x5               g064(.a(\a[15] ), .b(\b[14] ), .out0(new_n160));
  xorc02aa1n12x5               g065(.a(\a[16] ), .b(\b[15] ), .out0(new_n161));
  aoi112aa1n02x5               g066(.a(new_n161), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n162));
  aoai13aa1n02x5               g067(.a(new_n161), .b(new_n159), .c(new_n157), .d(new_n160), .o1(new_n163));
  norb02aa1n03x4               g068(.a(new_n163), .b(new_n162), .out0(\s[16] ));
  nand23aa1d12x5               g069(.a(new_n154), .b(new_n160), .c(new_n161), .o1(new_n165));
  nano22aa1n12x5               g070(.a(new_n165), .b(new_n131), .c(new_n140), .out0(new_n166));
  aoai13aa1n12x5               g071(.a(new_n166), .b(new_n122), .c(new_n105), .d(new_n116), .o1(new_n167));
  nanp02aa1n02x5               g072(.a(new_n161), .b(new_n160), .o1(new_n168));
  orn002aa1n02x5               g073(.a(\a[15] ), .b(\b[14] ), .o(new_n169));
  oao003aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .c(new_n169), .carry(new_n170));
  tech160nm_fioai012aa1n04x5   g075(.a(new_n170), .b(new_n168), .c(new_n156), .o1(new_n171));
  oab012aa1n12x5               g076(.a(new_n171), .b(new_n144), .c(new_n165), .out0(new_n172));
  nanp02aa1n09x5               g077(.a(new_n167), .b(new_n172), .o1(new_n173));
  xorb03aa1n02x5               g078(.a(new_n173), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g079(.a(\a[18] ), .o1(new_n175));
  inv000aa1d42x5               g080(.a(\a[17] ), .o1(new_n176));
  inv000aa1d42x5               g081(.a(\b[16] ), .o1(new_n177));
  oaoi03aa1n02x5               g082(.a(new_n176), .b(new_n177), .c(new_n173), .o1(new_n178));
  xorb03aa1n02x5               g083(.a(new_n178), .b(\b[17] ), .c(new_n175), .out0(\s[18] ));
  xroi22aa1d04x5               g084(.a(new_n176), .b(\b[16] ), .c(new_n175), .d(\b[17] ), .out0(new_n180));
  nanp02aa1n02x5               g085(.a(new_n177), .b(new_n176), .o1(new_n181));
  oaoi03aa1n02x5               g086(.a(\a[18] ), .b(\b[17] ), .c(new_n181), .o1(new_n182));
  nor042aa1n04x5               g087(.a(\b[18] ), .b(\a[19] ), .o1(new_n183));
  nand02aa1n04x5               g088(.a(\b[18] ), .b(\a[19] ), .o1(new_n184));
  norb02aa1n02x5               g089(.a(new_n184), .b(new_n183), .out0(new_n185));
  aoai13aa1n06x5               g090(.a(new_n185), .b(new_n182), .c(new_n173), .d(new_n180), .o1(new_n186));
  aoi112aa1n02x5               g091(.a(new_n185), .b(new_n182), .c(new_n173), .d(new_n180), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n186), .b(new_n187), .out0(\s[19] ));
  xnrc02aa1n02x5               g093(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor042aa1n04x5               g094(.a(\b[19] ), .b(\a[20] ), .o1(new_n190));
  nand02aa1n06x5               g095(.a(\b[19] ), .b(\a[20] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  nona22aa1n02x5               g097(.a(new_n186), .b(new_n192), .c(new_n183), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n192), .o1(new_n194));
  oaoi13aa1n06x5               g099(.a(new_n194), .b(new_n186), .c(\a[19] ), .d(\b[18] ), .o1(new_n195));
  norb02aa1n03x4               g100(.a(new_n193), .b(new_n195), .out0(\s[20] ));
  nano23aa1n03x7               g101(.a(new_n183), .b(new_n190), .c(new_n191), .d(new_n184), .out0(new_n197));
  nanp02aa1n02x5               g102(.a(new_n180), .b(new_n197), .o1(new_n198));
  oai022aa1n02x5               g103(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n199));
  oaib12aa1n06x5               g104(.a(new_n199), .b(new_n175), .c(\b[17] ), .out0(new_n200));
  nona23aa1n12x5               g105(.a(new_n191), .b(new_n184), .c(new_n183), .d(new_n190), .out0(new_n201));
  aoi012aa1n06x5               g106(.a(new_n190), .b(new_n183), .c(new_n191), .o1(new_n202));
  oai012aa1n18x5               g107(.a(new_n202), .b(new_n201), .c(new_n200), .o1(new_n203));
  inv000aa1d42x5               g108(.a(new_n203), .o1(new_n204));
  aoai13aa1n06x5               g109(.a(new_n204), .b(new_n198), .c(new_n167), .d(new_n172), .o1(new_n205));
  xorb03aa1n02x5               g110(.a(new_n205), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  nor042aa1n03x5               g111(.a(\b[20] ), .b(\a[21] ), .o1(new_n207));
  xorc02aa1n02x5               g112(.a(\a[21] ), .b(\b[20] ), .out0(new_n208));
  xorc02aa1n02x5               g113(.a(\a[22] ), .b(\b[21] ), .out0(new_n209));
  aoi112aa1n03x5               g114(.a(new_n207), .b(new_n209), .c(new_n205), .d(new_n208), .o1(new_n210));
  aoai13aa1n03x5               g115(.a(new_n209), .b(new_n207), .c(new_n205), .d(new_n208), .o1(new_n211));
  norb02aa1n03x4               g116(.a(new_n211), .b(new_n210), .out0(\s[22] ));
  inv000aa1d42x5               g117(.a(\a[21] ), .o1(new_n213));
  inv000aa1d42x5               g118(.a(\a[22] ), .o1(new_n214));
  xroi22aa1d04x5               g119(.a(new_n213), .b(\b[20] ), .c(new_n214), .d(\b[21] ), .out0(new_n215));
  nanp03aa1n02x5               g120(.a(new_n215), .b(new_n180), .c(new_n197), .o1(new_n216));
  inv000aa1d42x5               g121(.a(\b[21] ), .o1(new_n217));
  oaoi03aa1n12x5               g122(.a(new_n214), .b(new_n217), .c(new_n207), .o1(new_n218));
  inv000aa1n02x5               g123(.a(new_n218), .o1(new_n219));
  aoi012aa1n02x5               g124(.a(new_n219), .b(new_n203), .c(new_n215), .o1(new_n220));
  aoai13aa1n06x5               g125(.a(new_n220), .b(new_n216), .c(new_n167), .d(new_n172), .o1(new_n221));
  xorb03aa1n02x5               g126(.a(new_n221), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g127(.a(\b[22] ), .b(\a[23] ), .o1(new_n223));
  tech160nm_fixorc02aa1n02p5x5 g128(.a(\a[23] ), .b(\b[22] ), .out0(new_n224));
  xorc02aa1n02x5               g129(.a(\a[24] ), .b(\b[23] ), .out0(new_n225));
  aoi112aa1n03x5               g130(.a(new_n223), .b(new_n225), .c(new_n221), .d(new_n224), .o1(new_n226));
  aoai13aa1n03x5               g131(.a(new_n225), .b(new_n223), .c(new_n221), .d(new_n224), .o1(new_n227));
  norb02aa1n03x4               g132(.a(new_n227), .b(new_n226), .out0(\s[24] ));
  and002aa1n02x5               g133(.a(new_n225), .b(new_n224), .o(new_n229));
  inv000aa1n02x5               g134(.a(new_n229), .o1(new_n230));
  nano32aa1n02x4               g135(.a(new_n230), .b(new_n215), .c(new_n180), .d(new_n197), .out0(new_n231));
  inv000aa1n02x5               g136(.a(new_n202), .o1(new_n232));
  aoai13aa1n03x5               g137(.a(new_n215), .b(new_n232), .c(new_n197), .d(new_n182), .o1(new_n233));
  orn002aa1n02x5               g138(.a(\a[23] ), .b(\b[22] ), .o(new_n234));
  oao003aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .c(new_n234), .carry(new_n235));
  aoai13aa1n04x5               g140(.a(new_n235), .b(new_n230), .c(new_n233), .d(new_n218), .o1(new_n236));
  tech160nm_fixorc02aa1n04x5   g141(.a(\a[25] ), .b(\b[24] ), .out0(new_n237));
  aoai13aa1n06x5               g142(.a(new_n237), .b(new_n236), .c(new_n173), .d(new_n231), .o1(new_n238));
  aoi112aa1n02x5               g143(.a(new_n237), .b(new_n236), .c(new_n173), .d(new_n231), .o1(new_n239));
  norb02aa1n03x4               g144(.a(new_n238), .b(new_n239), .out0(\s[25] ));
  nor042aa1n03x5               g145(.a(\b[24] ), .b(\a[25] ), .o1(new_n241));
  tech160nm_fixorc02aa1n04x5   g146(.a(\a[26] ), .b(\b[25] ), .out0(new_n242));
  nona22aa1n02x5               g147(.a(new_n238), .b(new_n242), .c(new_n241), .out0(new_n243));
  inv000aa1d42x5               g148(.a(new_n241), .o1(new_n244));
  aobi12aa1n06x5               g149(.a(new_n242), .b(new_n238), .c(new_n244), .out0(new_n245));
  norb02aa1n03x4               g150(.a(new_n243), .b(new_n245), .out0(\s[26] ));
  oai122aa1n02x7               g151(.a(new_n170), .b(new_n144), .c(new_n165), .d(new_n168), .e(new_n156), .o1(new_n247));
  and002aa1n06x5               g152(.a(new_n242), .b(new_n237), .o(new_n248));
  nano22aa1n03x7               g153(.a(new_n216), .b(new_n229), .c(new_n248), .out0(new_n249));
  aoai13aa1n06x5               g154(.a(new_n249), .b(new_n247), .c(new_n128), .d(new_n166), .o1(new_n250));
  oao003aa1n02x5               g155(.a(\a[26] ), .b(\b[25] ), .c(new_n244), .carry(new_n251));
  aobi12aa1n06x5               g156(.a(new_n251), .b(new_n236), .c(new_n248), .out0(new_n252));
  xorc02aa1n02x5               g157(.a(\a[27] ), .b(\b[26] ), .out0(new_n253));
  xnbna2aa1n03x5               g158(.a(new_n253), .b(new_n252), .c(new_n250), .out0(\s[27] ));
  norp02aa1n02x5               g159(.a(\b[26] ), .b(\a[27] ), .o1(new_n255));
  inv040aa1n03x5               g160(.a(new_n255), .o1(new_n256));
  aobi12aa1n02x7               g161(.a(new_n253), .b(new_n252), .c(new_n250), .out0(new_n257));
  xnrc02aa1n02x5               g162(.a(\b[27] ), .b(\a[28] ), .out0(new_n258));
  nano22aa1n02x4               g163(.a(new_n257), .b(new_n256), .c(new_n258), .out0(new_n259));
  aobi12aa1n06x5               g164(.a(new_n249), .b(new_n167), .c(new_n172), .out0(new_n260));
  aoai13aa1n03x5               g165(.a(new_n229), .b(new_n219), .c(new_n203), .d(new_n215), .o1(new_n261));
  inv000aa1d42x5               g166(.a(new_n248), .o1(new_n262));
  aoai13aa1n06x5               g167(.a(new_n251), .b(new_n262), .c(new_n261), .d(new_n235), .o1(new_n263));
  oaih12aa1n02x5               g168(.a(new_n253), .b(new_n263), .c(new_n260), .o1(new_n264));
  aoi012aa1n03x5               g169(.a(new_n258), .b(new_n264), .c(new_n256), .o1(new_n265));
  nor002aa1n02x5               g170(.a(new_n265), .b(new_n259), .o1(\s[28] ));
  norb02aa1n02x5               g171(.a(new_n253), .b(new_n258), .out0(new_n267));
  aobi12aa1n03x5               g172(.a(new_n267), .b(new_n252), .c(new_n250), .out0(new_n268));
  oao003aa1n02x5               g173(.a(\a[28] ), .b(\b[27] ), .c(new_n256), .carry(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[28] ), .b(\a[29] ), .out0(new_n270));
  nano22aa1n03x5               g175(.a(new_n268), .b(new_n269), .c(new_n270), .out0(new_n271));
  oaih12aa1n02x5               g176(.a(new_n267), .b(new_n263), .c(new_n260), .o1(new_n272));
  aoi012aa1n03x5               g177(.a(new_n270), .b(new_n272), .c(new_n269), .o1(new_n273));
  norp02aa1n03x5               g178(.a(new_n273), .b(new_n271), .o1(\s[29] ));
  xorb03aa1n02x5               g179(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g180(.a(new_n253), .b(new_n270), .c(new_n258), .out0(new_n276));
  aobi12aa1n02x7               g181(.a(new_n276), .b(new_n252), .c(new_n250), .out0(new_n277));
  oao003aa1n02x5               g182(.a(\a[29] ), .b(\b[28] ), .c(new_n269), .carry(new_n278));
  xnrc02aa1n02x5               g183(.a(\b[29] ), .b(\a[30] ), .out0(new_n279));
  nano22aa1n02x4               g184(.a(new_n277), .b(new_n278), .c(new_n279), .out0(new_n280));
  oaih12aa1n02x5               g185(.a(new_n276), .b(new_n263), .c(new_n260), .o1(new_n281));
  aoi012aa1n03x5               g186(.a(new_n279), .b(new_n281), .c(new_n278), .o1(new_n282));
  nor002aa1n02x5               g187(.a(new_n282), .b(new_n280), .o1(\s[30] ));
  xnrc02aa1n02x5               g188(.a(\b[30] ), .b(\a[31] ), .out0(new_n284));
  norb02aa1n02x5               g189(.a(new_n276), .b(new_n279), .out0(new_n285));
  aobi12aa1n03x5               g190(.a(new_n285), .b(new_n252), .c(new_n250), .out0(new_n286));
  oao003aa1n02x5               g191(.a(\a[30] ), .b(\b[29] ), .c(new_n278), .carry(new_n287));
  nano22aa1n03x5               g192(.a(new_n286), .b(new_n284), .c(new_n287), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n285), .b(new_n263), .c(new_n260), .o1(new_n289));
  aoi012aa1n03x5               g194(.a(new_n284), .b(new_n289), .c(new_n287), .o1(new_n290));
  norp02aa1n03x5               g195(.a(new_n290), .b(new_n288), .o1(\s[31] ));
  xnrb03aa1n02x5               g196(.a(new_n109), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g197(.a(\a[3] ), .b(\b[2] ), .c(new_n109), .o1(new_n293));
  xorb03aa1n02x5               g198(.a(new_n293), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g199(.a(new_n116), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  nanb02aa1n02x5               g200(.a(new_n104), .b(new_n116), .out0(new_n296));
  tech160nm_fioai012aa1n03p5x5 g201(.a(new_n296), .b(\b[4] ), .c(\a[5] ), .o1(new_n297));
  xorb03aa1n02x5               g202(.a(new_n297), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  nanb02aa1n02x5               g203(.a(new_n100), .b(new_n101), .out0(new_n299));
  nanb02aa1n02x5               g204(.a(new_n103), .b(new_n297), .out0(new_n300));
  oaoi13aa1n06x5               g205(.a(new_n299), .b(new_n300), .c(\a[6] ), .d(\b[5] ), .o1(new_n301));
  oai112aa1n02x5               g206(.a(new_n300), .b(new_n299), .c(\b[5] ), .d(\a[6] ), .o1(new_n302));
  norb02aa1n02x5               g207(.a(new_n302), .b(new_n301), .out0(\s[7] ));
  nanb02aa1n02x5               g208(.a(new_n98), .b(new_n99), .out0(new_n304));
  oab012aa1n02x4               g209(.a(new_n304), .b(new_n301), .c(new_n100), .out0(new_n305));
  aoi112aa1n02x5               g210(.a(new_n301), .b(new_n100), .c(new_n117), .d(new_n99), .o1(new_n306));
  norp02aa1n02x5               g211(.a(new_n305), .b(new_n306), .o1(\s[8] ));
  xorb03aa1n02x5               g212(.a(new_n128), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


