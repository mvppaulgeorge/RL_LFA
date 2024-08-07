// Benchmark "adder" written by ABC on Thu Jul 18 11:06:26 2024

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
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n141,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n151, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n181,
    new_n182, new_n183, new_n184, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n197, new_n198,
    new_n199, new_n200, new_n201, new_n202, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n212, new_n213,
    new_n214, new_n215, new_n217, new_n218, new_n219, new_n220, new_n221,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n229,
    new_n230, new_n231, new_n232, new_n234, new_n235, new_n236, new_n237,
    new_n238, new_n240, new_n241, new_n242, new_n243, new_n244, new_n245,
    new_n246, new_n247, new_n248, new_n250, new_n251, new_n252, new_n253,
    new_n254, new_n255, new_n256, new_n258, new_n259, new_n260, new_n261,
    new_n262, new_n263, new_n264, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n274, new_n275, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n283, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n300, new_n303,
    new_n306, new_n308, new_n309, new_n311, new_n313;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  orn002aa1n02x5               g001(.a(\a[9] ), .b(\b[8] ), .o(new_n97));
  nor042aa1n04x5               g002(.a(\b[7] ), .b(\a[8] ), .o1(new_n98));
  nand42aa1n16x5               g003(.a(\b[7] ), .b(\a[8] ), .o1(new_n99));
  norb02aa1n06x5               g004(.a(new_n99), .b(new_n98), .out0(new_n100));
  xorc02aa1n12x5               g005(.a(\a[7] ), .b(\b[6] ), .out0(new_n101));
  nand42aa1n02x5               g006(.a(\b[5] ), .b(\a[6] ), .o1(new_n102));
  nor022aa1n16x5               g007(.a(\b[5] ), .b(\a[6] ), .o1(new_n103));
  norp02aa1n04x5               g008(.a(\b[4] ), .b(\a[5] ), .o1(new_n104));
  nand22aa1n03x5               g009(.a(\b[4] ), .b(\a[5] ), .o1(new_n105));
  nona23aa1n03x5               g010(.a(new_n102), .b(new_n105), .c(new_n104), .d(new_n103), .out0(new_n106));
  nano22aa1n03x7               g011(.a(new_n106), .b(new_n101), .c(new_n100), .out0(new_n107));
  nor022aa1n04x5               g012(.a(\b[1] ), .b(\a[2] ), .o1(new_n108));
  nand02aa1d04x5               g013(.a(\b[0] ), .b(\a[1] ), .o1(new_n109));
  nand22aa1n09x5               g014(.a(\b[1] ), .b(\a[2] ), .o1(new_n110));
  tech160nm_fiaoi012aa1n05x5   g015(.a(new_n108), .b(new_n109), .c(new_n110), .o1(new_n111));
  nor042aa1n06x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nanp02aa1n06x5               g017(.a(\b[3] ), .b(\a[4] ), .o1(new_n113));
  nor022aa1n16x5               g018(.a(\b[2] ), .b(\a[3] ), .o1(new_n114));
  nand42aa1n03x5               g019(.a(\b[2] ), .b(\a[3] ), .o1(new_n115));
  nona23aa1n09x5               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  oa0012aa1n02x5               g021(.a(new_n113), .b(new_n114), .c(new_n112), .o(new_n117));
  inv030aa1n03x5               g022(.a(new_n117), .o1(new_n118));
  oai012aa1n12x5               g023(.a(new_n118), .b(new_n116), .c(new_n111), .o1(new_n119));
  aoi112aa1n02x5               g024(.a(\b[6] ), .b(\a[7] ), .c(\a[8] ), .d(\b[7] ), .o1(new_n120));
  aoi112aa1n09x5               g025(.a(\b[4] ), .b(\a[5] ), .c(\a[6] ), .d(\b[5] ), .o1(new_n121));
  oai112aa1n03x5               g026(.a(new_n101), .b(new_n100), .c(new_n103), .d(new_n121), .o1(new_n122));
  nona22aa1n03x5               g027(.a(new_n122), .b(new_n120), .c(new_n98), .out0(new_n123));
  tech160nm_fixorc02aa1n03p5x5 g028(.a(\a[9] ), .b(\b[8] ), .out0(new_n124));
  aoai13aa1n02x5               g029(.a(new_n124), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n125));
  tech160nm_fixorc02aa1n03p5x5 g030(.a(\a[10] ), .b(\b[9] ), .out0(new_n126));
  xnbna2aa1n03x5               g031(.a(new_n126), .b(new_n125), .c(new_n97), .out0(\s[10] ));
  nor002aa1n03x5               g032(.a(\b[9] ), .b(\a[10] ), .o1(new_n128));
  and002aa1n02x5               g033(.a(new_n126), .b(new_n124), .o(new_n129));
  aoai13aa1n02x5               g034(.a(new_n129), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n130));
  aoi112aa1n09x5               g035(.a(\b[8] ), .b(\a[9] ), .c(\a[10] ), .d(\b[9] ), .o1(new_n131));
  nona22aa1n02x4               g036(.a(new_n130), .b(new_n131), .c(new_n128), .out0(new_n132));
  xorb03aa1n02x5               g037(.a(new_n132), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  nor002aa1n06x5               g038(.a(\b[10] ), .b(\a[11] ), .o1(new_n134));
  nand42aa1d28x5               g039(.a(\b[10] ), .b(\a[11] ), .o1(new_n135));
  aoi012aa1n02x5               g040(.a(new_n134), .b(new_n132), .c(new_n135), .o1(new_n136));
  nor002aa1n16x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nand42aa1d28x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n06x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  xnrc02aa1n02x5               g044(.a(new_n136), .b(new_n139), .out0(\s[12] ));
  nano23aa1n02x5               g045(.a(new_n134), .b(new_n137), .c(new_n138), .d(new_n135), .out0(new_n141));
  and003aa1n02x5               g046(.a(new_n141), .b(new_n126), .c(new_n124), .o(new_n142));
  aoai13aa1n06x5               g047(.a(new_n142), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n143));
  aoi112aa1n09x5               g048(.a(\b[10] ), .b(\a[11] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n144));
  norb02aa1n06x4               g049(.a(new_n135), .b(new_n134), .out0(new_n145));
  oai112aa1n06x5               g050(.a(new_n139), .b(new_n145), .c(new_n131), .d(new_n128), .o1(new_n146));
  nona22aa1d18x5               g051(.a(new_n146), .b(new_n144), .c(new_n137), .out0(new_n147));
  inv000aa1d42x5               g052(.a(new_n147), .o1(new_n148));
  nanp02aa1n02x5               g053(.a(new_n143), .b(new_n148), .o1(new_n149));
  xorb03aa1n02x5               g054(.a(new_n149), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  nor002aa1n04x5               g055(.a(\b[12] ), .b(\a[13] ), .o1(new_n151));
  nanp02aa1n04x5               g056(.a(\b[12] ), .b(\a[13] ), .o1(new_n152));
  aoi012aa1n02x5               g057(.a(new_n151), .b(new_n149), .c(new_n152), .o1(new_n153));
  xnrb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  norp02aa1n04x5               g059(.a(\b[13] ), .b(\a[14] ), .o1(new_n155));
  nand02aa1n04x5               g060(.a(\b[13] ), .b(\a[14] ), .o1(new_n156));
  nona23aa1n02x4               g061(.a(new_n156), .b(new_n152), .c(new_n151), .d(new_n155), .out0(new_n157));
  aoi012aa1n02x5               g062(.a(new_n155), .b(new_n151), .c(new_n156), .o1(new_n158));
  aoai13aa1n02x7               g063(.a(new_n158), .b(new_n157), .c(new_n143), .d(new_n148), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  nor022aa1n08x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nand42aa1n04x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  nor002aa1n06x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  tech160nm_finand02aa1n03p5x5 g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  aoi112aa1n02x5               g070(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n167));
  norb02aa1n02x7               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  nano23aa1n02x5               g073(.a(new_n151), .b(new_n155), .c(new_n156), .d(new_n152), .out0(new_n169));
  nano23aa1n03x7               g074(.a(new_n161), .b(new_n163), .c(new_n164), .d(new_n162), .out0(new_n170));
  nand02aa1n02x5               g075(.a(new_n170), .b(new_n169), .o1(new_n171));
  nano32aa1n03x7               g076(.a(new_n171), .b(new_n141), .c(new_n126), .d(new_n124), .out0(new_n172));
  aoai13aa1n06x5               g077(.a(new_n172), .b(new_n123), .c(new_n119), .d(new_n107), .o1(new_n173));
  nona23aa1n03x5               g078(.a(new_n164), .b(new_n162), .c(new_n161), .d(new_n163), .out0(new_n174));
  nor042aa1n02x5               g079(.a(new_n174), .b(new_n157), .o1(new_n175));
  aoi112aa1n02x5               g080(.a(\b[14] ), .b(\a[15] ), .c(\a[16] ), .d(\b[15] ), .o1(new_n176));
  oai022aa1n02x5               g081(.a(new_n174), .b(new_n158), .c(\b[15] ), .d(\a[16] ), .o1(new_n177));
  aoi112aa1n09x5               g082(.a(new_n177), .b(new_n176), .c(new_n147), .d(new_n175), .o1(new_n178));
  nand02aa1d10x5               g083(.a(new_n178), .b(new_n173), .o1(new_n179));
  xorb03aa1n02x5               g084(.a(new_n179), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g085(.a(\a[18] ), .o1(new_n181));
  inv000aa1d42x5               g086(.a(\a[17] ), .o1(new_n182));
  inv000aa1d42x5               g087(.a(\b[16] ), .o1(new_n183));
  oaoi03aa1n03x5               g088(.a(new_n182), .b(new_n183), .c(new_n179), .o1(new_n184));
  xorb03aa1n02x5               g089(.a(new_n184), .b(\b[17] ), .c(new_n181), .out0(\s[18] ));
  xroi22aa1d06x4               g090(.a(new_n182), .b(\b[16] ), .c(new_n181), .d(\b[17] ), .out0(new_n186));
  nanp02aa1n02x5               g091(.a(\b[17] ), .b(\a[18] ), .o1(new_n187));
  nona22aa1n02x4               g092(.a(new_n187), .b(\b[16] ), .c(\a[17] ), .out0(new_n188));
  oaib12aa1n02x5               g093(.a(new_n188), .b(\b[17] ), .c(new_n181), .out0(new_n189));
  nor042aa1n06x5               g094(.a(\b[18] ), .b(\a[19] ), .o1(new_n190));
  nand42aa1n02x5               g095(.a(\b[18] ), .b(\a[19] ), .o1(new_n191));
  norb02aa1n02x5               g096(.a(new_n191), .b(new_n190), .out0(new_n192));
  aoai13aa1n06x5               g097(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n193));
  aoi112aa1n02x5               g098(.a(new_n192), .b(new_n189), .c(new_n179), .d(new_n186), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n193), .b(new_n194), .out0(\s[19] ));
  xnrc02aa1n02x5               g100(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  nor022aa1n04x5               g101(.a(\b[19] ), .b(\a[20] ), .o1(new_n197));
  nanp02aa1n02x5               g102(.a(\b[19] ), .b(\a[20] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n198), .b(new_n197), .out0(new_n199));
  nona22aa1n03x5               g104(.a(new_n193), .b(new_n199), .c(new_n190), .out0(new_n200));
  inv000aa1n06x5               g105(.a(new_n190), .o1(new_n201));
  aobi12aa1n02x7               g106(.a(new_n199), .b(new_n193), .c(new_n201), .out0(new_n202));
  norb02aa1n03x4               g107(.a(new_n200), .b(new_n202), .out0(\s[20] ));
  nano23aa1n06x5               g108(.a(new_n190), .b(new_n197), .c(new_n198), .d(new_n191), .out0(new_n204));
  nand02aa1d06x5               g109(.a(new_n186), .b(new_n204), .o1(new_n205));
  inv000aa1d42x5               g110(.a(new_n205), .o1(new_n206));
  norp02aa1n02x5               g111(.a(\b[17] ), .b(\a[18] ), .o1(new_n207));
  aoi013aa1n02x4               g112(.a(new_n207), .b(new_n187), .c(new_n182), .d(new_n183), .o1(new_n208));
  nona23aa1n02x4               g113(.a(new_n198), .b(new_n191), .c(new_n190), .d(new_n197), .out0(new_n209));
  oaoi03aa1n03x5               g114(.a(\a[20] ), .b(\b[19] ), .c(new_n201), .o1(new_n210));
  inv020aa1n02x5               g115(.a(new_n210), .o1(new_n211));
  oai012aa1n06x5               g116(.a(new_n211), .b(new_n209), .c(new_n208), .o1(new_n212));
  xorc02aa1n02x5               g117(.a(\a[21] ), .b(\b[20] ), .out0(new_n213));
  aoai13aa1n06x5               g118(.a(new_n213), .b(new_n212), .c(new_n179), .d(new_n206), .o1(new_n214));
  aoi112aa1n02x5               g119(.a(new_n213), .b(new_n212), .c(new_n179), .d(new_n206), .o1(new_n215));
  norb02aa1n02x5               g120(.a(new_n214), .b(new_n215), .out0(\s[21] ));
  norp02aa1n04x5               g121(.a(\b[20] ), .b(\a[21] ), .o1(new_n217));
  inv040aa1n03x5               g122(.a(new_n217), .o1(new_n218));
  tech160nm_fixnrc02aa1n05x5   g123(.a(\b[21] ), .b(\a[22] ), .out0(new_n219));
  nand03aa1n02x5               g124(.a(new_n214), .b(new_n218), .c(new_n219), .o1(new_n220));
  aoi012aa1n02x7               g125(.a(new_n219), .b(new_n214), .c(new_n218), .o1(new_n221));
  norb02aa1n02x7               g126(.a(new_n220), .b(new_n221), .out0(\s[22] ));
  nanp02aa1n02x5               g127(.a(\b[20] ), .b(\a[21] ), .o1(new_n223));
  nano22aa1n03x7               g128(.a(new_n219), .b(new_n218), .c(new_n223), .out0(new_n224));
  and003aa1n02x5               g129(.a(new_n186), .b(new_n224), .c(new_n204), .o(new_n225));
  aoai13aa1n06x5               g130(.a(new_n224), .b(new_n210), .c(new_n204), .d(new_n189), .o1(new_n226));
  oaoi03aa1n02x5               g131(.a(\a[22] ), .b(\b[21] ), .c(new_n218), .o1(new_n227));
  inv000aa1n02x5               g132(.a(new_n227), .o1(new_n228));
  nanp02aa1n02x5               g133(.a(new_n226), .b(new_n228), .o1(new_n229));
  xorc02aa1n02x5               g134(.a(\a[23] ), .b(\b[22] ), .out0(new_n230));
  aoai13aa1n06x5               g135(.a(new_n230), .b(new_n229), .c(new_n179), .d(new_n225), .o1(new_n231));
  aoi112aa1n02x5               g136(.a(new_n230), .b(new_n229), .c(new_n179), .d(new_n225), .o1(new_n232));
  norb02aa1n02x5               g137(.a(new_n231), .b(new_n232), .out0(\s[23] ));
  nor042aa1n03x5               g138(.a(\b[22] ), .b(\a[23] ), .o1(new_n234));
  xorc02aa1n02x5               g139(.a(\a[24] ), .b(\b[23] ), .out0(new_n235));
  nona22aa1n03x5               g140(.a(new_n231), .b(new_n235), .c(new_n234), .out0(new_n236));
  inv000aa1n02x5               g141(.a(new_n234), .o1(new_n237));
  aobi12aa1n02x7               g142(.a(new_n235), .b(new_n231), .c(new_n237), .out0(new_n238));
  norb02aa1n03x4               g143(.a(new_n236), .b(new_n238), .out0(\s[24] ));
  inv000aa1d42x5               g144(.a(\a[23] ), .o1(new_n240));
  inv020aa1n04x5               g145(.a(\a[24] ), .o1(new_n241));
  xroi22aa1d06x4               g146(.a(new_n240), .b(\b[22] ), .c(new_n241), .d(\b[23] ), .out0(new_n242));
  nanb03aa1n03x5               g147(.a(new_n205), .b(new_n242), .c(new_n224), .out0(new_n243));
  inv000aa1n02x5               g148(.a(new_n242), .o1(new_n244));
  oao003aa1n02x5               g149(.a(\a[24] ), .b(\b[23] ), .c(new_n237), .carry(new_n245));
  aoai13aa1n04x5               g150(.a(new_n245), .b(new_n244), .c(new_n226), .d(new_n228), .o1(new_n246));
  inv040aa1n03x5               g151(.a(new_n246), .o1(new_n247));
  aoai13aa1n06x5               g152(.a(new_n247), .b(new_n243), .c(new_n173), .d(new_n178), .o1(new_n248));
  xorb03aa1n02x5               g153(.a(new_n248), .b(\b[24] ), .c(\a[25] ), .out0(\s[25] ));
  norp02aa1n02x5               g154(.a(\b[24] ), .b(\a[25] ), .o1(new_n250));
  xorc02aa1n12x5               g155(.a(\a[25] ), .b(\b[24] ), .out0(new_n251));
  nor042aa1n02x5               g156(.a(\b[25] ), .b(\a[26] ), .o1(new_n252));
  nanp02aa1n04x5               g157(.a(\b[25] ), .b(\a[26] ), .o1(new_n253));
  norb02aa1n06x5               g158(.a(new_n253), .b(new_n252), .out0(new_n254));
  aoi112aa1n02x5               g159(.a(new_n250), .b(new_n254), .c(new_n248), .d(new_n251), .o1(new_n255));
  aoai13aa1n03x5               g160(.a(new_n254), .b(new_n250), .c(new_n248), .d(new_n251), .o1(new_n256));
  norb02aa1n03x4               g161(.a(new_n256), .b(new_n255), .out0(\s[26] ));
  tech160nm_finand02aa1n05x5   g162(.a(new_n251), .b(new_n254), .o1(new_n258));
  inv000aa1n02x5               g163(.a(new_n258), .o1(new_n259));
  nano32aa1d12x5               g164(.a(new_n205), .b(new_n259), .c(new_n224), .d(new_n242), .out0(new_n260));
  nand02aa1d10x5               g165(.a(new_n179), .b(new_n260), .o1(new_n261));
  oai012aa1n02x5               g166(.a(new_n253), .b(new_n252), .c(new_n250), .o1(new_n262));
  aobi12aa1n06x5               g167(.a(new_n262), .b(new_n246), .c(new_n259), .out0(new_n263));
  xorc02aa1n12x5               g168(.a(\a[27] ), .b(\b[26] ), .out0(new_n264));
  xnbna2aa1n06x5               g169(.a(new_n264), .b(new_n263), .c(new_n261), .out0(\s[27] ));
  nor042aa1n03x5               g170(.a(\b[26] ), .b(\a[27] ), .o1(new_n266));
  inv040aa1n03x5               g171(.a(new_n266), .o1(new_n267));
  aobi12aa1n06x5               g172(.a(new_n264), .b(new_n263), .c(new_n261), .out0(new_n268));
  xnrc02aa1n02x5               g173(.a(\b[27] ), .b(\a[28] ), .out0(new_n269));
  nano22aa1n03x7               g174(.a(new_n268), .b(new_n267), .c(new_n269), .out0(new_n270));
  aobi12aa1n06x5               g175(.a(new_n260), .b(new_n178), .c(new_n173), .out0(new_n271));
  aoai13aa1n04x5               g176(.a(new_n242), .b(new_n227), .c(new_n212), .d(new_n224), .o1(new_n272));
  aoai13aa1n06x5               g177(.a(new_n262), .b(new_n258), .c(new_n272), .d(new_n245), .o1(new_n273));
  oaih12aa1n02x5               g178(.a(new_n264), .b(new_n273), .c(new_n271), .o1(new_n274));
  tech160nm_fiaoi012aa1n02p5x5 g179(.a(new_n269), .b(new_n274), .c(new_n267), .o1(new_n275));
  norp02aa1n03x5               g180(.a(new_n275), .b(new_n270), .o1(\s[28] ));
  norb02aa1n02x5               g181(.a(new_n264), .b(new_n269), .out0(new_n277));
  aobi12aa1n06x5               g182(.a(new_n277), .b(new_n263), .c(new_n261), .out0(new_n278));
  oao003aa1n02x5               g183(.a(\a[28] ), .b(\b[27] ), .c(new_n267), .carry(new_n279));
  xnrc02aa1n02x5               g184(.a(\b[28] ), .b(\a[29] ), .out0(new_n280));
  nano22aa1n03x7               g185(.a(new_n278), .b(new_n279), .c(new_n280), .out0(new_n281));
  oaih12aa1n02x5               g186(.a(new_n277), .b(new_n273), .c(new_n271), .o1(new_n282));
  tech160nm_fiaoi012aa1n02p5x5 g187(.a(new_n280), .b(new_n282), .c(new_n279), .o1(new_n283));
  norp02aa1n03x5               g188(.a(new_n283), .b(new_n281), .o1(\s[29] ));
  xorb03aa1n02x5               g189(.a(new_n109), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g190(.a(new_n264), .b(new_n280), .c(new_n269), .out0(new_n286));
  aobi12aa1n06x5               g191(.a(new_n286), .b(new_n263), .c(new_n261), .out0(new_n287));
  oao003aa1n02x5               g192(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n288));
  xnrc02aa1n02x5               g193(.a(\b[29] ), .b(\a[30] ), .out0(new_n289));
  nano22aa1n03x7               g194(.a(new_n287), .b(new_n288), .c(new_n289), .out0(new_n290));
  oaih12aa1n02x5               g195(.a(new_n286), .b(new_n273), .c(new_n271), .o1(new_n291));
  tech160nm_fiaoi012aa1n02p5x5 g196(.a(new_n289), .b(new_n291), .c(new_n288), .o1(new_n292));
  norp02aa1n03x5               g197(.a(new_n292), .b(new_n290), .o1(\s[30] ));
  norb02aa1n02x5               g198(.a(new_n286), .b(new_n289), .out0(new_n294));
  aobi12aa1n06x5               g199(.a(new_n294), .b(new_n263), .c(new_n261), .out0(new_n295));
  oao003aa1n02x5               g200(.a(\a[30] ), .b(\b[29] ), .c(new_n288), .carry(new_n296));
  xnrc02aa1n02x5               g201(.a(\b[30] ), .b(\a[31] ), .out0(new_n297));
  nano22aa1n03x7               g202(.a(new_n295), .b(new_n296), .c(new_n297), .out0(new_n298));
  oaih12aa1n02x5               g203(.a(new_n294), .b(new_n273), .c(new_n271), .o1(new_n299));
  tech160nm_fiaoi012aa1n02p5x5 g204(.a(new_n297), .b(new_n299), .c(new_n296), .o1(new_n300));
  norp02aa1n03x5               g205(.a(new_n300), .b(new_n298), .o1(\s[31] ));
  xnrb03aa1n02x5               g206(.a(new_n111), .b(\b[2] ), .c(\a[3] ), .out0(\s[3] ));
  oaoi03aa1n02x5               g207(.a(\a[3] ), .b(\b[2] ), .c(new_n111), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g209(.a(new_n119), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  tech160nm_fiao0012aa1n02p5x5 g210(.a(new_n104), .b(new_n119), .c(new_n105), .o(new_n306));
  xorb03aa1n02x5               g211(.a(new_n306), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  aoai13aa1n02x5               g212(.a(new_n101), .b(new_n103), .c(new_n306), .d(new_n102), .o1(new_n308));
  aoi112aa1n02x5               g213(.a(new_n103), .b(new_n101), .c(new_n306), .d(new_n102), .o1(new_n309));
  norb02aa1n02x5               g214(.a(new_n308), .b(new_n309), .out0(\s[7] ));
  orn002aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .o(new_n311));
  xnbna2aa1n03x5               g216(.a(new_n100), .b(new_n308), .c(new_n311), .out0(\s[8] ));
  aoi112aa1n02x5               g217(.a(new_n123), .b(new_n124), .c(new_n119), .d(new_n107), .o1(new_n313));
  norb02aa1n02x5               g218(.a(new_n125), .b(new_n313), .out0(\s[9] ));
endmodule


