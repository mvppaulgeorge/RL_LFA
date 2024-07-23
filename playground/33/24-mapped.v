// Benchmark "adder" written by ABC on Thu Jul 18 05:04:22 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n133,
    new_n134, new_n135, new_n136, new_n137, new_n138, new_n139, new_n140,
    new_n142, new_n143, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n154, new_n155, new_n156,
    new_n157, new_n159, new_n160, new_n161, new_n162, new_n163, new_n164,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n177, new_n178, new_n179, new_n180,
    new_n181, new_n183, new_n184, new_n185, new_n186, new_n188, new_n189,
    new_n190, new_n191, new_n192, new_n193, new_n194, new_n195, new_n196,
    new_n197, new_n200, new_n201, new_n202, new_n203, new_n205, new_n206,
    new_n207, new_n208, new_n209, new_n210, new_n211, new_n213, new_n214,
    new_n215, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n225, new_n226, new_n227, new_n228, new_n230,
    new_n231, new_n232, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n243, new_n244, new_n246,
    new_n247, new_n248, new_n249, new_n250, new_n251, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n272, new_n273, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n280, new_n281, new_n282, new_n285, new_n286,
    new_n287, new_n288, new_n289, new_n290, new_n291, new_n292, new_n294,
    new_n295, new_n296, new_n297, new_n298, new_n299, new_n302, new_n303,
    new_n304, new_n307, new_n309, new_n311;
  inv000aa1d42x5               g000(.a(\a[0] ), .o1(\s[0] ));
  inv040aa1d32x5               g001(.a(\a[3] ), .o1(new_n97));
  inv000aa1d42x5               g002(.a(\b[2] ), .o1(new_n98));
  nanp02aa1n02x5               g003(.a(new_n98), .b(new_n97), .o1(new_n99));
  nanp02aa1n02x5               g004(.a(\b[2] ), .b(\a[3] ), .o1(new_n100));
  nand42aa1n02x5               g005(.a(new_n99), .b(new_n100), .o1(new_n101));
  orn002aa1n02x5               g006(.a(\a[2] ), .b(\b[1] ), .o(new_n102));
  tech160nm_finand02aa1n05x5   g007(.a(\b[0] ), .b(\a[1] ), .o1(new_n103));
  aob012aa1n03x5               g008(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(new_n104));
  inv000aa1d42x5               g009(.a(\a[4] ), .o1(new_n105));
  aboi22aa1n03x5               g010(.a(\b[3] ), .b(new_n105), .c(new_n97), .d(new_n98), .out0(new_n106));
  aoai13aa1n12x5               g011(.a(new_n106), .b(new_n101), .c(new_n104), .d(new_n102), .o1(new_n107));
  nanp02aa1n02x5               g012(.a(\b[5] ), .b(\a[6] ), .o1(new_n108));
  inv000aa1d42x5               g013(.a(\a[6] ), .o1(new_n109));
  inv000aa1d42x5               g014(.a(\b[5] ), .o1(new_n110));
  nanp02aa1n02x5               g015(.a(new_n110), .b(new_n109), .o1(new_n111));
  nand42aa1n02x5               g016(.a(\b[3] ), .b(\a[4] ), .o1(new_n112));
  nor042aa1n04x5               g017(.a(\b[4] ), .b(\a[5] ), .o1(new_n113));
  nand42aa1n02x5               g018(.a(\b[4] ), .b(\a[5] ), .o1(new_n114));
  nanb02aa1n02x5               g019(.a(new_n113), .b(new_n114), .out0(new_n115));
  nano32aa1n03x7               g020(.a(new_n115), .b(new_n111), .c(new_n112), .d(new_n108), .out0(new_n116));
  nand42aa1n08x5               g021(.a(\b[7] ), .b(\a[8] ), .o1(new_n117));
  norp02aa1n04x5               g022(.a(\b[7] ), .b(\a[8] ), .o1(new_n118));
  nor002aa1n02x5               g023(.a(\b[6] ), .b(\a[7] ), .o1(new_n119));
  nand42aa1n04x5               g024(.a(\b[6] ), .b(\a[7] ), .o1(new_n120));
  nano23aa1n03x7               g025(.a(new_n119), .b(new_n118), .c(new_n120), .d(new_n117), .out0(new_n121));
  nona23aa1n02x4               g026(.a(new_n117), .b(new_n120), .c(new_n119), .d(new_n118), .out0(new_n122));
  oaoi03aa1n02x5               g027(.a(new_n109), .b(new_n110), .c(new_n113), .o1(new_n123));
  aoi012aa1n02x5               g028(.a(new_n118), .b(new_n119), .c(new_n117), .o1(new_n124));
  oai012aa1n03x5               g029(.a(new_n124), .b(new_n122), .c(new_n123), .o1(new_n125));
  aoi013aa1n09x5               g030(.a(new_n125), .b(new_n107), .c(new_n116), .d(new_n121), .o1(new_n126));
  tech160nm_fioaoi03aa1n03p5x5 g031(.a(\a[9] ), .b(\b[8] ), .c(new_n126), .o1(new_n127));
  xorb03aa1n02x5               g032(.a(new_n127), .b(\b[9] ), .c(\a[10] ), .out0(\s[10] ));
  nor022aa1n04x5               g033(.a(\b[9] ), .b(\a[10] ), .o1(new_n129));
  nand02aa1n04x5               g034(.a(\b[9] ), .b(\a[10] ), .o1(new_n130));
  tech160nm_fioai012aa1n05x5   g035(.a(new_n130), .b(new_n127), .c(new_n129), .o1(new_n131));
  xnrb03aa1n03x5               g036(.a(new_n131), .b(\b[10] ), .c(\a[11] ), .out0(\s[11] ));
  oaoi03aa1n02x5               g037(.a(\a[11] ), .b(\b[10] ), .c(new_n131), .o1(new_n133));
  xnrc02aa1n12x5               g038(.a(\b[11] ), .b(\a[12] ), .out0(new_n134));
  nand42aa1n03x5               g039(.a(new_n133), .b(new_n134), .o1(new_n135));
  tech160nm_finor002aa1n03p5x5 g040(.a(\b[10] ), .b(\a[11] ), .o1(new_n136));
  nand02aa1d04x5               g041(.a(\b[10] ), .b(\a[11] ), .o1(new_n137));
  nanb02aa1n02x5               g042(.a(new_n136), .b(new_n137), .out0(new_n138));
  orn002aa1n03x5               g043(.a(new_n131), .b(new_n138), .o(new_n139));
  nona22aa1n02x4               g044(.a(new_n139), .b(new_n134), .c(new_n136), .out0(new_n140));
  nanp02aa1n02x5               g045(.a(new_n140), .b(new_n135), .o1(\s[12] ));
  nona23aa1n09x5               g046(.a(new_n137), .b(new_n130), .c(new_n136), .d(new_n129), .out0(new_n142));
  xnrc02aa1n02x5               g047(.a(\b[8] ), .b(\a[9] ), .out0(new_n143));
  nor043aa1d12x5               g048(.a(new_n142), .b(new_n143), .c(new_n134), .o1(new_n144));
  inv000aa1d42x5               g049(.a(new_n144), .o1(new_n145));
  oai022aa1n02x5               g050(.a(\a[10] ), .b(\b[9] ), .c(\b[8] ), .d(\a[9] ), .o1(new_n146));
  nanp03aa1n02x5               g051(.a(new_n146), .b(new_n130), .c(new_n137), .o1(new_n147));
  oai122aa1n02x7               g052(.a(new_n147), .b(\a[12] ), .c(\b[11] ), .d(\a[11] ), .e(\b[10] ), .o1(new_n148));
  aob012aa1n02x5               g053(.a(new_n148), .b(\b[11] ), .c(\a[12] ), .out0(new_n149));
  tech160nm_fioai012aa1n05x5   g054(.a(new_n149), .b(new_n126), .c(new_n145), .o1(new_n150));
  xorb03aa1n02x5               g055(.a(new_n150), .b(\b[12] ), .c(\a[13] ), .out0(\s[13] ));
  norp02aa1n04x5               g056(.a(\b[13] ), .b(\a[14] ), .o1(new_n152));
  nand02aa1n06x5               g057(.a(\b[13] ), .b(\a[14] ), .o1(new_n153));
  norb02aa1n06x4               g058(.a(new_n153), .b(new_n152), .out0(new_n154));
  nor042aa1n04x5               g059(.a(\b[12] ), .b(\a[13] ), .o1(new_n155));
  nand42aa1n06x5               g060(.a(\b[12] ), .b(\a[13] ), .o1(new_n156));
  aoi012aa1n02x5               g061(.a(new_n155), .b(new_n150), .c(new_n156), .o1(new_n157));
  xnrc02aa1n02x5               g062(.a(new_n157), .b(new_n154), .out0(\s[14] ));
  nano23aa1n03x7               g063(.a(new_n155), .b(new_n152), .c(new_n153), .d(new_n156), .out0(new_n159));
  aoi012aa1n02x5               g064(.a(new_n152), .b(new_n155), .c(new_n153), .o1(new_n160));
  inv000aa1d42x5               g065(.a(new_n160), .o1(new_n161));
  xorc02aa1n02x5               g066(.a(\a[15] ), .b(\b[14] ), .out0(new_n162));
  aoai13aa1n06x5               g067(.a(new_n162), .b(new_n161), .c(new_n150), .d(new_n159), .o1(new_n163));
  aoi112aa1n02x5               g068(.a(new_n162), .b(new_n161), .c(new_n150), .d(new_n159), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n163), .b(new_n164), .out0(\s[15] ));
  orn002aa1n02x5               g070(.a(\a[15] ), .b(\b[14] ), .o(new_n166));
  xorc02aa1n02x5               g071(.a(\a[16] ), .b(\b[15] ), .out0(new_n167));
  xnbna2aa1n03x5               g072(.a(new_n167), .b(new_n163), .c(new_n166), .out0(\s[16] ));
  nanp02aa1n02x5               g073(.a(\b[14] ), .b(\a[15] ), .o1(new_n169));
  orn002aa1n02x5               g074(.a(\a[16] ), .b(\b[15] ), .o(new_n170));
  and002aa1n02x5               g075(.a(\b[15] ), .b(\a[16] ), .o(new_n171));
  nano32aa1n02x4               g076(.a(new_n171), .b(new_n166), .c(new_n170), .d(new_n169), .out0(new_n172));
  nand23aa1n06x5               g077(.a(new_n144), .b(new_n159), .c(new_n172), .o1(new_n173));
  norb02aa1n02x5               g078(.a(new_n156), .b(new_n155), .out0(new_n174));
  nanp02aa1n02x5               g079(.a(new_n167), .b(new_n162), .o1(new_n175));
  aoi022aa1n02x5               g080(.a(\b[15] ), .b(\a[16] ), .c(\a[12] ), .d(\b[11] ), .o1(new_n176));
  nano32aa1n02x4               g081(.a(new_n175), .b(new_n154), .c(new_n174), .d(new_n176), .out0(new_n177));
  aoai13aa1n02x5               g082(.a(new_n169), .b(new_n152), .c(new_n155), .d(new_n153), .o1(new_n178));
  aoai13aa1n02x5               g083(.a(new_n170), .b(new_n171), .c(new_n178), .d(new_n166), .o1(new_n179));
  aoi012aa1n06x5               g084(.a(new_n179), .b(new_n177), .c(new_n148), .o1(new_n180));
  oai012aa1d24x5               g085(.a(new_n180), .b(new_n126), .c(new_n173), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  inv000aa1d42x5               g087(.a(\a[18] ), .o1(new_n183));
  inv000aa1d42x5               g088(.a(\a[17] ), .o1(new_n184));
  inv000aa1d42x5               g089(.a(\b[16] ), .o1(new_n185));
  oaoi03aa1n02x5               g090(.a(new_n184), .b(new_n185), .c(new_n181), .o1(new_n186));
  xorb03aa1n02x5               g091(.a(new_n186), .b(\b[17] ), .c(new_n183), .out0(\s[18] ));
  xroi22aa1d06x4               g092(.a(new_n184), .b(\b[16] ), .c(new_n183), .d(\b[17] ), .out0(new_n188));
  inv000aa1d42x5               g093(.a(\b[17] ), .o1(new_n189));
  oaih22aa1n04x5               g094(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n190));
  oaib12aa1n12x5               g095(.a(new_n190), .b(new_n189), .c(\a[18] ), .out0(new_n191));
  inv000aa1d42x5               g096(.a(new_n191), .o1(new_n192));
  nor022aa1n16x5               g097(.a(\b[18] ), .b(\a[19] ), .o1(new_n193));
  nand42aa1n02x5               g098(.a(\b[18] ), .b(\a[19] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  aoai13aa1n06x5               g100(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n196));
  aoi112aa1n02x5               g101(.a(new_n195), .b(new_n192), .c(new_n181), .d(new_n188), .o1(new_n197));
  norb02aa1n02x5               g102(.a(new_n196), .b(new_n197), .out0(\s[19] ));
  xnrc02aa1n02x5               g103(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  inv000aa1d42x5               g104(.a(new_n193), .o1(new_n200));
  tech160nm_finor002aa1n05x5   g105(.a(\b[19] ), .b(\a[20] ), .o1(new_n201));
  nand42aa1n02x5               g106(.a(\b[19] ), .b(\a[20] ), .o1(new_n202));
  norb02aa1n02x5               g107(.a(new_n202), .b(new_n201), .out0(new_n203));
  xnbna2aa1n03x5               g108(.a(new_n203), .b(new_n196), .c(new_n200), .out0(\s[20] ));
  nona23aa1n02x4               g109(.a(new_n202), .b(new_n194), .c(new_n193), .d(new_n201), .out0(new_n205));
  norb02aa1n02x5               g110(.a(new_n188), .b(new_n205), .out0(new_n206));
  aoi012aa1n02x5               g111(.a(new_n201), .b(new_n193), .c(new_n202), .o1(new_n207));
  tech160nm_fioai012aa1n04x5   g112(.a(new_n207), .b(new_n205), .c(new_n191), .o1(new_n208));
  xorc02aa1n02x5               g113(.a(\a[21] ), .b(\b[20] ), .out0(new_n209));
  aoai13aa1n06x5               g114(.a(new_n209), .b(new_n208), .c(new_n181), .d(new_n206), .o1(new_n210));
  aoi112aa1n02x5               g115(.a(new_n209), .b(new_n208), .c(new_n181), .d(new_n206), .o1(new_n211));
  norb02aa1n02x5               g116(.a(new_n210), .b(new_n211), .out0(\s[21] ));
  inv040aa1d30x5               g117(.a(\a[21] ), .o1(new_n213));
  nanb02aa1n02x5               g118(.a(\b[20] ), .b(new_n213), .out0(new_n214));
  xnrc02aa1n02x5               g119(.a(\b[21] ), .b(\a[22] ), .out0(new_n215));
  xobna2aa1n03x5               g120(.a(new_n215), .b(new_n210), .c(new_n214), .out0(\s[22] ));
  nano23aa1n06x5               g121(.a(new_n193), .b(new_n201), .c(new_n202), .d(new_n194), .out0(new_n217));
  inv040aa1d32x5               g122(.a(\a[22] ), .o1(new_n218));
  xroi22aa1d06x4               g123(.a(new_n213), .b(\b[20] ), .c(new_n218), .d(\b[21] ), .out0(new_n219));
  nand23aa1n06x5               g124(.a(new_n219), .b(new_n188), .c(new_n217), .o1(new_n220));
  inv040aa1n03x5               g125(.a(new_n220), .o1(new_n221));
  oao003aa1n12x5               g126(.a(\a[22] ), .b(\b[21] ), .c(new_n214), .carry(new_n222));
  inv000aa1d42x5               g127(.a(new_n222), .o1(new_n223));
  aoi012aa1n02x5               g128(.a(new_n223), .b(new_n208), .c(new_n219), .o1(new_n224));
  inv030aa1n03x5               g129(.a(new_n224), .o1(new_n225));
  xorc02aa1n12x5               g130(.a(\a[23] ), .b(\b[22] ), .out0(new_n226));
  aoai13aa1n06x5               g131(.a(new_n226), .b(new_n225), .c(new_n181), .d(new_n221), .o1(new_n227));
  aoi112aa1n02x5               g132(.a(new_n226), .b(new_n225), .c(new_n181), .d(new_n221), .o1(new_n228));
  norb02aa1n02x5               g133(.a(new_n227), .b(new_n228), .out0(\s[23] ));
  inv000aa1d42x5               g134(.a(\a[23] ), .o1(new_n230));
  nanb02aa1n02x5               g135(.a(\b[22] ), .b(new_n230), .out0(new_n231));
  tech160nm_fixnrc02aa1n05x5   g136(.a(\b[23] ), .b(\a[24] ), .out0(new_n232));
  xobna2aa1n03x5               g137(.a(new_n232), .b(new_n227), .c(new_n231), .out0(\s[24] ));
  inv000aa1n02x5               g138(.a(new_n207), .o1(new_n234));
  aoai13aa1n06x5               g139(.a(new_n219), .b(new_n234), .c(new_n217), .d(new_n192), .o1(new_n235));
  norb02aa1n03x5               g140(.a(new_n226), .b(new_n232), .out0(new_n236));
  inv000aa1n02x5               g141(.a(new_n236), .o1(new_n237));
  oao003aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .c(new_n231), .carry(new_n238));
  aoai13aa1n04x5               g143(.a(new_n238), .b(new_n237), .c(new_n235), .d(new_n222), .o1(new_n239));
  inv030aa1n02x5               g144(.a(new_n239), .o1(new_n240));
  inv000aa1n02x5               g145(.a(new_n206), .o1(new_n241));
  inv000aa1d42x5               g146(.a(new_n219), .o1(new_n242));
  nona32aa1n09x5               g147(.a(new_n181), .b(new_n237), .c(new_n242), .d(new_n241), .out0(new_n243));
  xnrc02aa1n12x5               g148(.a(\b[24] ), .b(\a[25] ), .out0(new_n244));
  xobna2aa1n03x5               g149(.a(new_n244), .b(new_n243), .c(new_n240), .out0(\s[25] ));
  nor042aa1n03x5               g150(.a(\b[24] ), .b(\a[25] ), .o1(new_n246));
  tech160nm_fiaoi012aa1n04x5   g151(.a(new_n244), .b(new_n243), .c(new_n240), .o1(new_n247));
  xnrc02aa1n02x5               g152(.a(\b[25] ), .b(\a[26] ), .out0(new_n248));
  oai012aa1n03x5               g153(.a(new_n248), .b(new_n247), .c(new_n246), .o1(new_n249));
  ao0012aa1n03x7               g154(.a(new_n244), .b(new_n243), .c(new_n240), .o(new_n250));
  nona22aa1n03x5               g155(.a(new_n250), .b(new_n248), .c(new_n246), .out0(new_n251));
  nanp02aa1n03x5               g156(.a(new_n251), .b(new_n249), .o1(\s[26] ));
  norp02aa1n02x5               g157(.a(new_n248), .b(new_n244), .o1(new_n253));
  inv000aa1d42x5               g158(.a(new_n246), .o1(new_n254));
  oao003aa1n02x5               g159(.a(\a[26] ), .b(\b[25] ), .c(new_n254), .carry(new_n255));
  aobi12aa1n06x5               g160(.a(new_n255), .b(new_n239), .c(new_n253), .out0(new_n256));
  inv000aa1n02x5               g161(.a(new_n253), .o1(new_n257));
  nona32aa1n09x5               g162(.a(new_n181), .b(new_n257), .c(new_n237), .d(new_n220), .out0(new_n258));
  xorc02aa1n02x5               g163(.a(\a[27] ), .b(\b[26] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n256), .c(new_n258), .out0(\s[27] ));
  nand42aa1n04x5               g165(.a(new_n256), .b(new_n258), .o1(new_n261));
  norp02aa1n02x5               g166(.a(\b[26] ), .b(\a[27] ), .o1(new_n262));
  norp02aa1n02x5               g167(.a(\b[27] ), .b(\a[28] ), .o1(new_n263));
  nanp02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .o1(new_n264));
  norb02aa1n09x5               g169(.a(new_n264), .b(new_n263), .out0(new_n265));
  inv000aa1d42x5               g170(.a(new_n265), .o1(new_n266));
  aoai13aa1n03x5               g171(.a(new_n266), .b(new_n262), .c(new_n261), .d(new_n259), .o1(new_n267));
  oaoi13aa1n04x5               g172(.a(new_n220), .b(new_n180), .c(new_n126), .d(new_n173), .o1(new_n268));
  aoai13aa1n06x5               g173(.a(new_n236), .b(new_n223), .c(new_n208), .d(new_n219), .o1(new_n269));
  aoai13aa1n04x5               g174(.a(new_n255), .b(new_n257), .c(new_n269), .d(new_n238), .o1(new_n270));
  nano22aa1n02x4               g175(.a(new_n232), .b(new_n253), .c(new_n226), .out0(new_n271));
  aoai13aa1n03x5               g176(.a(new_n259), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n272));
  nona22aa1n02x5               g177(.a(new_n272), .b(new_n266), .c(new_n262), .out0(new_n273));
  nanp02aa1n03x5               g178(.a(new_n267), .b(new_n273), .o1(\s[28] ));
  xorc02aa1n02x5               g179(.a(\a[29] ), .b(\b[28] ), .out0(new_n275));
  inv000aa1d42x5               g180(.a(new_n275), .o1(new_n276));
  norb02aa1n02x5               g181(.a(new_n259), .b(new_n266), .out0(new_n277));
  aoai13aa1n03x5               g182(.a(new_n277), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n278));
  oai012aa1n02x5               g183(.a(new_n264), .b(new_n263), .c(new_n262), .o1(new_n279));
  tech160nm_fiaoi012aa1n05x5   g184(.a(new_n276), .b(new_n278), .c(new_n279), .o1(new_n280));
  aobi12aa1n06x5               g185(.a(new_n277), .b(new_n256), .c(new_n258), .out0(new_n281));
  nano22aa1n03x5               g186(.a(new_n281), .b(new_n276), .c(new_n279), .out0(new_n282));
  norp02aa1n03x5               g187(.a(new_n280), .b(new_n282), .o1(\s[29] ));
  xorb03aa1n02x5               g188(.a(new_n103), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  nano22aa1n02x4               g189(.a(new_n276), .b(new_n259), .c(new_n265), .out0(new_n285));
  aoai13aa1n03x5               g190(.a(new_n285), .b(new_n270), .c(new_n268), .d(new_n271), .o1(new_n286));
  xorc02aa1n02x5               g191(.a(\a[30] ), .b(\b[29] ), .out0(new_n287));
  norp02aa1n02x5               g192(.a(\b[28] ), .b(\a[29] ), .o1(new_n288));
  aoi012aa1n02x5               g193(.a(new_n279), .b(\a[29] ), .c(\b[28] ), .o1(new_n289));
  norp03aa1n02x5               g194(.a(new_n289), .b(new_n287), .c(new_n288), .o1(new_n290));
  oao003aa1n02x5               g195(.a(\a[29] ), .b(\b[28] ), .c(new_n279), .carry(new_n291));
  tech160nm_finand02aa1n03p5x5 g196(.a(new_n286), .b(new_n291), .o1(new_n292));
  aoi022aa1n02x7               g197(.a(new_n292), .b(new_n287), .c(new_n286), .d(new_n290), .o1(\s[30] ));
  nanp03aa1n02x5               g198(.a(new_n277), .b(new_n275), .c(new_n287), .o1(new_n294));
  nanb02aa1n02x5               g199(.a(new_n294), .b(new_n261), .out0(new_n295));
  xorc02aa1n02x5               g200(.a(\a[31] ), .b(\b[30] ), .out0(new_n296));
  oao003aa1n02x5               g201(.a(\a[30] ), .b(\b[29] ), .c(new_n291), .carry(new_n297));
  norb02aa1n02x5               g202(.a(new_n297), .b(new_n296), .out0(new_n298));
  aoai13aa1n02x7               g203(.a(new_n297), .b(new_n294), .c(new_n256), .d(new_n258), .o1(new_n299));
  aoi022aa1n03x5               g204(.a(new_n295), .b(new_n298), .c(new_n299), .d(new_n296), .o1(\s[31] ));
  xobna2aa1n03x5               g205(.a(new_n101), .b(new_n104), .c(new_n102), .out0(\s[3] ));
  nanb02aa1n02x5               g206(.a(\b[3] ), .b(new_n105), .out0(new_n302));
  aoi012aa1n02x5               g207(.a(new_n101), .b(new_n104), .c(new_n102), .o1(new_n303));
  aoi122aa1n02x5               g208(.a(new_n303), .b(new_n112), .c(new_n302), .d(new_n98), .e(new_n97), .o1(new_n304));
  aoi013aa1n02x4               g209(.a(new_n304), .b(new_n112), .c(new_n107), .d(new_n302), .o1(\s[4] ));
  xnbna2aa1n03x5               g210(.a(new_n115), .b(new_n107), .c(new_n112), .out0(\s[5] ));
  aoi013aa1n02x4               g211(.a(new_n113), .b(new_n107), .c(new_n112), .d(new_n114), .o1(new_n307));
  xnbna2aa1n03x5               g212(.a(new_n307), .b(new_n108), .c(new_n111), .out0(\s[6] ));
  aob012aa1n02x5               g213(.a(new_n108), .b(new_n307), .c(new_n111), .out0(new_n309));
  xnrb03aa1n02x5               g214(.a(new_n309), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g215(.a(\a[7] ), .b(\b[6] ), .c(new_n309), .o1(new_n311));
  xorb03aa1n02x5               g216(.a(new_n311), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnrb03aa1n02x5               g217(.a(new_n126), .b(\b[8] ), .c(\a[9] ), .out0(\s[9] ));
endmodule


