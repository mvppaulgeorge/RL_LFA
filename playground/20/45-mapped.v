// Benchmark "adder" written by ABC on Wed Jul 17 22:37:44 2024

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
    new_n133, new_n135, new_n136, new_n137, new_n138, new_n140, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n151, new_n152, new_n153, new_n154, new_n155,
    new_n157, new_n158, new_n159, new_n161, new_n162, new_n163, new_n164,
    new_n165, new_n166, new_n167, new_n168, new_n169, new_n171, new_n172,
    new_n173, new_n174, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n199, new_n200, new_n201, new_n202, new_n203, new_n204, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n233, new_n234, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n249, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n265, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n276,
    new_n277, new_n279, new_n280, new_n281, new_n282, new_n283, new_n284,
    new_n285, new_n288, new_n289, new_n290, new_n291, new_n292, new_n293,
    new_n294, new_n296, new_n297, new_n298, new_n299, new_n300, new_n301,
    new_n302, new_n304, new_n306, new_n309, new_n311, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  nand22aa1n09x5               g001(.a(\b[0] ), .b(\a[1] ), .o1(new_n97));
  nand02aa1n03x5               g002(.a(\b[1] ), .b(\a[2] ), .o1(new_n98));
  nor042aa1n03x5               g003(.a(\b[1] ), .b(\a[2] ), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  oai112aa1n03x5               g005(.a(new_n100), .b(new_n98), .c(new_n99), .d(new_n97), .o1(new_n101));
  oa0022aa1n02x5               g006(.a(\a[4] ), .b(\b[3] ), .c(\a[3] ), .d(\b[2] ), .o(new_n102));
  aoi022aa1n06x5               g007(.a(new_n101), .b(new_n102), .c(\b[3] ), .d(\a[4] ), .o1(new_n103));
  nor002aa1n02x5               g008(.a(\b[5] ), .b(\a[6] ), .o1(new_n104));
  nanp02aa1n02x5               g009(.a(\b[5] ), .b(\a[6] ), .o1(new_n105));
  nor022aa1n04x5               g010(.a(\b[4] ), .b(\a[5] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[4] ), .b(\a[5] ), .o1(new_n107));
  nona23aa1n03x5               g012(.a(new_n107), .b(new_n105), .c(new_n104), .d(new_n106), .out0(new_n108));
  norp02aa1n02x5               g013(.a(\b[7] ), .b(\a[8] ), .o1(new_n109));
  nand22aa1n03x5               g014(.a(\b[7] ), .b(\a[8] ), .o1(new_n110));
  nor002aa1d32x5               g015(.a(\b[6] ), .b(\a[7] ), .o1(new_n111));
  nand22aa1n12x5               g016(.a(\b[6] ), .b(\a[7] ), .o1(new_n112));
  nona23aa1n09x5               g017(.a(new_n112), .b(new_n110), .c(new_n109), .d(new_n111), .out0(new_n113));
  nor042aa1n02x5               g018(.a(new_n113), .b(new_n108), .o1(new_n114));
  norb02aa1n06x4               g019(.a(new_n112), .b(new_n111), .out0(new_n115));
  oaih22aa1n04x5               g020(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n116));
  aoi022aa1n02x7               g021(.a(\b[7] ), .b(\a[8] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n117));
  nanp03aa1n06x5               g022(.a(new_n115), .b(new_n116), .c(new_n117), .o1(new_n118));
  inv040aa1n03x5               g023(.a(new_n111), .o1(new_n119));
  oaoi03aa1n02x5               g024(.a(\a[8] ), .b(\b[7] ), .c(new_n119), .o1(new_n120));
  nanb02aa1n12x5               g025(.a(new_n120), .b(new_n118), .out0(new_n121));
  nor042aa1n02x5               g026(.a(\b[8] ), .b(\a[9] ), .o1(new_n122));
  nanp02aa1n02x5               g027(.a(\b[8] ), .b(\a[9] ), .o1(new_n123));
  norb02aa1n03x5               g028(.a(new_n123), .b(new_n122), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n121), .c(new_n103), .d(new_n114), .o1(new_n125));
  oai012aa1n02x5               g030(.a(new_n125), .b(\b[8] ), .c(\a[9] ), .o1(new_n126));
  xorb03aa1n02x5               g031(.a(new_n126), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nanp02aa1n06x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  nor042aa1n04x5               g033(.a(\b[10] ), .b(\a[11] ), .o1(new_n129));
  nand42aa1n08x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  norb02aa1n02x5               g035(.a(new_n130), .b(new_n129), .out0(new_n131));
  nor002aa1n02x5               g036(.a(\b[9] ), .b(\a[10] ), .o1(new_n132));
  nona22aa1n02x4               g037(.a(new_n125), .b(new_n122), .c(new_n132), .out0(new_n133));
  xobna2aa1n03x5               g038(.a(new_n131), .b(new_n133), .c(new_n128), .out0(\s[11] ));
  inv000aa1d42x5               g039(.a(\b[11] ), .o1(new_n135));
  nanb02aa1n02x5               g040(.a(\a[12] ), .b(new_n135), .out0(new_n136));
  nand02aa1d12x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  aoi013aa1n02x4               g042(.a(new_n129), .b(new_n133), .c(new_n131), .d(new_n128), .o1(new_n138));
  xnbna2aa1n03x5               g043(.a(new_n138), .b(new_n136), .c(new_n137), .out0(\s[12] ));
  nanp02aa1n02x5               g044(.a(new_n103), .b(new_n114), .o1(new_n140));
  inv000aa1d42x5               g045(.a(new_n121), .o1(new_n141));
  nor022aa1n12x5               g046(.a(\b[11] ), .b(\a[12] ), .o1(new_n142));
  nano22aa1n03x5               g047(.a(new_n142), .b(new_n130), .c(new_n137), .out0(new_n143));
  norb03aa1n03x5               g048(.a(new_n128), .b(new_n132), .c(new_n129), .out0(new_n144));
  nand23aa1n03x5               g049(.a(new_n143), .b(new_n144), .c(new_n124), .o1(new_n145));
  nanb03aa1d24x5               g050(.a(new_n142), .b(new_n137), .c(new_n130), .out0(new_n146));
  inv040aa1d32x5               g051(.a(\a[11] ), .o1(new_n147));
  inv000aa1d42x5               g052(.a(\b[10] ), .o1(new_n148));
  nanp02aa1n04x5               g053(.a(new_n148), .b(new_n147), .o1(new_n149));
  oaih22aa1d12x5               g054(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n150));
  nand23aa1n09x5               g055(.a(new_n150), .b(new_n149), .c(new_n128), .o1(new_n151));
  aoi012aa1n09x5               g056(.a(new_n142), .b(new_n129), .c(new_n137), .o1(new_n152));
  oai012aa1d24x5               g057(.a(new_n152), .b(new_n151), .c(new_n146), .o1(new_n153));
  inv000aa1d42x5               g058(.a(new_n153), .o1(new_n154));
  aoai13aa1n06x5               g059(.a(new_n154), .b(new_n145), .c(new_n140), .d(new_n141), .o1(new_n155));
  xorb03aa1n02x5               g060(.a(new_n155), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor042aa1n02x5               g061(.a(\b[12] ), .b(\a[13] ), .o1(new_n157));
  nand42aa1n03x5               g062(.a(\b[12] ), .b(\a[13] ), .o1(new_n158));
  aoi012aa1n02x5               g063(.a(new_n157), .b(new_n155), .c(new_n158), .o1(new_n159));
  xnrb03aa1n02x5               g064(.a(new_n159), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  nor002aa1n02x5               g065(.a(\b[13] ), .b(\a[14] ), .o1(new_n161));
  nand42aa1n03x5               g066(.a(\b[13] ), .b(\a[14] ), .o1(new_n162));
  nano23aa1n06x5               g067(.a(new_n157), .b(new_n161), .c(new_n162), .d(new_n158), .out0(new_n163));
  oa0012aa1n06x5               g068(.a(new_n162), .b(new_n161), .c(new_n157), .o(new_n164));
  nor002aa1n16x5               g069(.a(\b[14] ), .b(\a[15] ), .o1(new_n165));
  nand42aa1n03x5               g070(.a(\b[14] ), .b(\a[15] ), .o1(new_n166));
  norb02aa1n02x5               g071(.a(new_n166), .b(new_n165), .out0(new_n167));
  aoai13aa1n02x5               g072(.a(new_n167), .b(new_n164), .c(new_n155), .d(new_n163), .o1(new_n168));
  aoi112aa1n02x5               g073(.a(new_n167), .b(new_n164), .c(new_n155), .d(new_n163), .o1(new_n169));
  norb02aa1n02x5               g074(.a(new_n168), .b(new_n169), .out0(\s[15] ));
  inv000aa1d42x5               g075(.a(new_n165), .o1(new_n171));
  nor042aa1n04x5               g076(.a(\b[15] ), .b(\a[16] ), .o1(new_n172));
  nand42aa1n10x5               g077(.a(\b[15] ), .b(\a[16] ), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n173), .b(new_n172), .out0(new_n174));
  xnbna2aa1n03x5               g079(.a(new_n174), .b(new_n168), .c(new_n171), .out0(\s[16] ));
  nano23aa1n06x5               g080(.a(new_n165), .b(new_n172), .c(new_n173), .d(new_n166), .out0(new_n176));
  nano22aa1n03x7               g081(.a(new_n145), .b(new_n163), .c(new_n176), .out0(new_n177));
  aoai13aa1n12x5               g082(.a(new_n177), .b(new_n121), .c(new_n103), .d(new_n114), .o1(new_n178));
  aoai13aa1n12x5               g083(.a(new_n176), .b(new_n164), .c(new_n153), .d(new_n163), .o1(new_n179));
  aoi012aa1d24x5               g084(.a(new_n172), .b(new_n165), .c(new_n173), .o1(new_n180));
  nand23aa1d12x5               g085(.a(new_n178), .b(new_n179), .c(new_n180), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv040aa1d32x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  nanp02aa1n02x5               g093(.a(new_n185), .b(new_n184), .o1(new_n189));
  oaoi03aa1n02x5               g094(.a(\a[18] ), .b(\b[17] ), .c(new_n189), .o1(new_n190));
  nor022aa1n16x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  nand42aa1n02x5               g096(.a(\b[18] ), .b(\a[19] ), .o1(new_n192));
  nanb02aa1n02x5               g097(.a(new_n191), .b(new_n192), .out0(new_n193));
  inv000aa1d42x5               g098(.a(new_n193), .o1(new_n194));
  aoai13aa1n06x5               g099(.a(new_n194), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n195));
  aoi112aa1n02x5               g100(.a(new_n194), .b(new_n190), .c(new_n181), .d(new_n188), .o1(new_n196));
  norb02aa1n02x5               g101(.a(new_n195), .b(new_n196), .out0(\s[19] ));
  xnrc02aa1n02x5               g102(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g103(.a(new_n191), .o1(new_n199));
  nor022aa1n06x5               g104(.a(\b[19] ), .b(\a[20] ), .o1(new_n200));
  nanp02aa1n04x5               g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nanb02aa1n02x5               g106(.a(new_n200), .b(new_n201), .out0(new_n202));
  nanp03aa1n03x5               g107(.a(new_n195), .b(new_n199), .c(new_n202), .o1(new_n203));
  aoi012aa1n06x5               g108(.a(new_n202), .b(new_n195), .c(new_n199), .o1(new_n204));
  norb02aa1n03x4               g109(.a(new_n203), .b(new_n204), .out0(\s[20] ));
  nona23aa1n02x4               g110(.a(new_n201), .b(new_n192), .c(new_n191), .d(new_n200), .out0(new_n206));
  norb02aa1n02x5               g111(.a(new_n188), .b(new_n206), .out0(new_n207));
  oai022aa1n02x5               g112(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n208));
  oaib12aa1n02x5               g113(.a(new_n208), .b(new_n183), .c(\b[17] ), .out0(new_n209));
  aoi012aa1n06x5               g114(.a(new_n200), .b(new_n191), .c(new_n201), .o1(new_n210));
  tech160nm_fioai012aa1n05x5   g115(.a(new_n210), .b(new_n206), .c(new_n209), .o1(new_n211));
  xnrc02aa1n12x5               g116(.a(\b[20] ), .b(\a[21] ), .out0(new_n212));
  inv000aa1d42x5               g117(.a(new_n212), .o1(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n211), .c(new_n181), .d(new_n207), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n211), .c(new_n181), .d(new_n207), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  tech160nm_fixnrc02aa1n04x5   g121(.a(\b[21] ), .b(\a[22] ), .out0(new_n217));
  oai112aa1n02x7               g122(.a(new_n214), .b(new_n217), .c(\b[20] ), .d(\a[21] ), .o1(new_n218));
  oaoi13aa1n06x5               g123(.a(new_n217), .b(new_n214), .c(\a[21] ), .d(\b[20] ), .o1(new_n219));
  norb02aa1n03x4               g124(.a(new_n218), .b(new_n219), .out0(\s[22] ));
  nano23aa1n09x5               g125(.a(new_n191), .b(new_n200), .c(new_n201), .d(new_n192), .out0(new_n221));
  nor042aa1n04x5               g126(.a(new_n217), .b(new_n212), .o1(new_n222));
  and003aa1n12x5               g127(.a(new_n188), .b(new_n222), .c(new_n221), .o(new_n223));
  inv000aa1n03x5               g128(.a(new_n210), .o1(new_n224));
  aoai13aa1n06x5               g129(.a(new_n222), .b(new_n224), .c(new_n221), .d(new_n190), .o1(new_n225));
  inv000aa1d42x5               g130(.a(\a[22] ), .o1(new_n226));
  inv000aa1d42x5               g131(.a(\b[21] ), .o1(new_n227));
  norp02aa1n02x5               g132(.a(\b[20] ), .b(\a[21] ), .o1(new_n228));
  oaoi03aa1n12x5               g133(.a(new_n226), .b(new_n227), .c(new_n228), .o1(new_n229));
  nanp02aa1n02x5               g134(.a(new_n225), .b(new_n229), .o1(new_n230));
  xnrc02aa1n12x5               g135(.a(\b[22] ), .b(\a[23] ), .out0(new_n231));
  inv000aa1d42x5               g136(.a(new_n231), .o1(new_n232));
  aoai13aa1n06x5               g137(.a(new_n232), .b(new_n230), .c(new_n181), .d(new_n223), .o1(new_n233));
  aoi112aa1n02x5               g138(.a(new_n232), .b(new_n230), .c(new_n181), .d(new_n223), .o1(new_n234));
  norb02aa1n02x5               g139(.a(new_n233), .b(new_n234), .out0(\s[23] ));
  tech160nm_fixnrc02aa1n04x5   g140(.a(\b[23] ), .b(\a[24] ), .out0(new_n236));
  oai112aa1n03x5               g141(.a(new_n233), .b(new_n236), .c(\b[22] ), .d(\a[23] ), .o1(new_n237));
  oaoi13aa1n06x5               g142(.a(new_n236), .b(new_n233), .c(\a[23] ), .d(\b[22] ), .o1(new_n238));
  norb02aa1n03x4               g143(.a(new_n237), .b(new_n238), .out0(\s[24] ));
  nor042aa1n04x5               g144(.a(new_n236), .b(new_n231), .o1(new_n240));
  inv000aa1d42x5               g145(.a(new_n240), .o1(new_n241));
  nano32aa1n02x4               g146(.a(new_n241), .b(new_n188), .c(new_n222), .d(new_n221), .out0(new_n242));
  norp02aa1n02x5               g147(.a(\b[23] ), .b(\a[24] ), .o1(new_n243));
  aoi112aa1n02x5               g148(.a(\b[22] ), .b(\a[23] ), .c(\a[24] ), .d(\b[23] ), .o1(new_n244));
  norp02aa1n02x5               g149(.a(new_n244), .b(new_n243), .o1(new_n245));
  aoai13aa1n06x5               g150(.a(new_n245), .b(new_n241), .c(new_n225), .d(new_n229), .o1(new_n246));
  aoi012aa1n02x5               g151(.a(new_n246), .b(new_n181), .c(new_n242), .o1(new_n247));
  xnrc02aa1n12x5               g152(.a(\b[24] ), .b(\a[25] ), .out0(new_n248));
  inv000aa1d42x5               g153(.a(new_n248), .o1(new_n249));
  xnrc02aa1n02x5               g154(.a(new_n247), .b(new_n249), .out0(\s[25] ));
  nor042aa1n03x5               g155(.a(\b[24] ), .b(\a[25] ), .o1(new_n251));
  inv000aa1d42x5               g156(.a(new_n251), .o1(new_n252));
  aoai13aa1n06x5               g157(.a(new_n249), .b(new_n246), .c(new_n181), .d(new_n242), .o1(new_n253));
  tech160nm_fixnrc02aa1n02p5x5 g158(.a(\b[25] ), .b(\a[26] ), .out0(new_n254));
  nanp03aa1n03x5               g159(.a(new_n253), .b(new_n252), .c(new_n254), .o1(new_n255));
  tech160nm_fiaoi012aa1n02p5x5 g160(.a(new_n254), .b(new_n253), .c(new_n252), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n255), .b(new_n256), .out0(\s[26] ));
  norp02aa1n06x5               g162(.a(new_n254), .b(new_n248), .o1(new_n258));
  inv020aa1n03x5               g163(.a(new_n258), .o1(new_n259));
  nona32aa1d18x5               g164(.a(new_n223), .b(new_n259), .c(new_n236), .d(new_n231), .out0(new_n260));
  inv000aa1d42x5               g165(.a(new_n260), .o1(new_n261));
  nand22aa1n12x5               g166(.a(new_n181), .b(new_n261), .o1(new_n262));
  oao003aa1n02x5               g167(.a(\a[26] ), .b(\b[25] ), .c(new_n252), .carry(new_n263));
  aobi12aa1n12x5               g168(.a(new_n263), .b(new_n246), .c(new_n258), .out0(new_n264));
  xorc02aa1n02x5               g169(.a(\a[27] ), .b(\b[26] ), .out0(new_n265));
  xnbna2aa1n03x5               g170(.a(new_n265), .b(new_n264), .c(new_n262), .out0(\s[27] ));
  nor042aa1n03x5               g171(.a(\b[26] ), .b(\a[27] ), .o1(new_n267));
  inv000aa1d42x5               g172(.a(new_n267), .o1(new_n268));
  aobi12aa1n03x5               g173(.a(new_n265), .b(new_n264), .c(new_n262), .out0(new_n269));
  xnrc02aa1n02x5               g174(.a(\b[27] ), .b(\a[28] ), .out0(new_n270));
  nano22aa1n03x5               g175(.a(new_n269), .b(new_n268), .c(new_n270), .out0(new_n271));
  aoi013aa1n06x4               g176(.a(new_n260), .b(new_n178), .c(new_n179), .d(new_n180), .o1(new_n272));
  inv000aa1d42x5               g177(.a(new_n229), .o1(new_n273));
  aoai13aa1n03x5               g178(.a(new_n240), .b(new_n273), .c(new_n211), .d(new_n222), .o1(new_n274));
  aoai13aa1n06x5               g179(.a(new_n263), .b(new_n259), .c(new_n274), .d(new_n245), .o1(new_n275));
  oaih12aa1n02x5               g180(.a(new_n265), .b(new_n275), .c(new_n272), .o1(new_n276));
  tech160nm_fiaoi012aa1n03p5x5 g181(.a(new_n270), .b(new_n276), .c(new_n268), .o1(new_n277));
  nor002aa1n02x5               g182(.a(new_n277), .b(new_n271), .o1(\s[28] ));
  norb02aa1n02x5               g183(.a(new_n265), .b(new_n270), .out0(new_n279));
  oaih12aa1n02x5               g184(.a(new_n279), .b(new_n275), .c(new_n272), .o1(new_n280));
  oao003aa1n02x5               g185(.a(\a[28] ), .b(\b[27] ), .c(new_n268), .carry(new_n281));
  xnrc02aa1n02x5               g186(.a(\b[28] ), .b(\a[29] ), .out0(new_n282));
  aoi012aa1n06x5               g187(.a(new_n282), .b(new_n280), .c(new_n281), .o1(new_n283));
  aobi12aa1n06x5               g188(.a(new_n279), .b(new_n264), .c(new_n262), .out0(new_n284));
  nano22aa1n03x7               g189(.a(new_n284), .b(new_n281), .c(new_n282), .out0(new_n285));
  norp02aa1n03x5               g190(.a(new_n283), .b(new_n285), .o1(\s[29] ));
  xorb03aa1n02x5               g191(.a(new_n97), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g192(.a(new_n265), .b(new_n282), .c(new_n270), .out0(new_n288));
  oaih12aa1n02x5               g193(.a(new_n288), .b(new_n275), .c(new_n272), .o1(new_n289));
  oao003aa1n02x5               g194(.a(\a[29] ), .b(\b[28] ), .c(new_n281), .carry(new_n290));
  xnrc02aa1n02x5               g195(.a(\b[29] ), .b(\a[30] ), .out0(new_n291));
  tech160nm_fiaoi012aa1n03p5x5 g196(.a(new_n291), .b(new_n289), .c(new_n290), .o1(new_n292));
  aobi12aa1n06x5               g197(.a(new_n288), .b(new_n264), .c(new_n262), .out0(new_n293));
  nano22aa1n03x7               g198(.a(new_n293), .b(new_n290), .c(new_n291), .out0(new_n294));
  norp02aa1n03x5               g199(.a(new_n292), .b(new_n294), .o1(\s[30] ));
  norb02aa1n02x5               g200(.a(new_n288), .b(new_n291), .out0(new_n296));
  oaih12aa1n02x5               g201(.a(new_n296), .b(new_n275), .c(new_n272), .o1(new_n297));
  oao003aa1n02x5               g202(.a(\a[30] ), .b(\b[29] ), .c(new_n290), .carry(new_n298));
  xnrc02aa1n02x5               g203(.a(\b[30] ), .b(\a[31] ), .out0(new_n299));
  aoi012aa1n06x5               g204(.a(new_n299), .b(new_n297), .c(new_n298), .o1(new_n300));
  aobi12aa1n03x5               g205(.a(new_n296), .b(new_n264), .c(new_n262), .out0(new_n301));
  nano22aa1n03x5               g206(.a(new_n301), .b(new_n298), .c(new_n299), .out0(new_n302));
  norp02aa1n03x5               g207(.a(new_n300), .b(new_n302), .o1(\s[31] ));
  oai012aa1n02x5               g208(.a(new_n98), .b(new_n99), .c(new_n97), .o1(new_n304));
  xnrb03aa1n02x5               g209(.a(new_n304), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g210(.a(\a[3] ), .b(\b[2] ), .c(new_n304), .o1(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g212(.a(new_n103), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  oai012aa1n02x5               g213(.a(new_n107), .b(new_n103), .c(new_n106), .o1(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oaib12aa1n02x5               g215(.a(new_n105), .b(new_n104), .c(new_n309), .out0(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n311), .b(new_n119), .c(new_n112), .out0(\s[7] ));
  oaoi03aa1n02x5               g217(.a(\a[7] ), .b(\b[6] ), .c(new_n311), .o1(new_n313));
  xorb03aa1n02x5               g218(.a(new_n313), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g219(.a(new_n124), .b(new_n140), .c(new_n141), .out0(\s[9] ));
endmodule


