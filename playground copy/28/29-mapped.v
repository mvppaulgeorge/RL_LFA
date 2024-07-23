// Benchmark "adder" written by ABC on Thu Jul 11 12:59:22 2024

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
    new_n125, new_n126, new_n127, new_n129, new_n130, new_n131, new_n132,
    new_n133, new_n134, new_n135, new_n137, new_n138, new_n139, new_n140,
    new_n141, new_n142, new_n144, new_n145, new_n146, new_n147, new_n148,
    new_n149, new_n150, new_n152, new_n153, new_n155, new_n156, new_n157,
    new_n158, new_n159, new_n161, new_n162, new_n163, new_n164, new_n165,
    new_n166, new_n167, new_n169, new_n170, new_n171, new_n172, new_n173,
    new_n174, new_n175, new_n176, new_n178, new_n179, new_n180, new_n181,
    new_n183, new_n184, new_n185, new_n186, new_n187, new_n188, new_n189,
    new_n190, new_n193, new_n194, new_n195, new_n196, new_n197, new_n198,
    new_n200, new_n201, new_n202, new_n203, new_n204, new_n205, new_n206,
    new_n207, new_n208, new_n210, new_n211, new_n212, new_n213, new_n214,
    new_n216, new_n217, new_n218, new_n219, new_n220, new_n221, new_n222,
    new_n223, new_n224, new_n226, new_n227, new_n228, new_n229, new_n230,
    new_n232, new_n233, new_n234, new_n235, new_n236, new_n237, new_n238,
    new_n239, new_n240, new_n241, new_n242, new_n244, new_n245, new_n246,
    new_n247, new_n248, new_n250, new_n251, new_n252, new_n253, new_n254,
    new_n255, new_n256, new_n257, new_n258, new_n259, new_n261, new_n262,
    new_n263, new_n264, new_n265, new_n266, new_n267, new_n268, new_n269,
    new_n270, new_n271, new_n273, new_n274, new_n275, new_n276, new_n277,
    new_n278, new_n279, new_n282, new_n283, new_n284, new_n285, new_n286,
    new_n287, new_n288, new_n290, new_n291, new_n292, new_n293, new_n294,
    new_n295, new_n296, new_n299, new_n302, new_n303, new_n305, new_n307;
  160nm_ficinv00aa1n08x5       g000(.clk(\a[0] ), .clkout(\s[0] ));
  xorc02aa1n02x5               g001(.a(\a[10] ), .b(\b[9] ), .out0(new_n97));
  orn002aa1n02x5               g002(.a(\a[9] ), .b(\b[8] ), .o(new_n98));
  and002aa1n02x5               g003(.a(\b[3] ), .b(\a[4] ), .o(new_n99));
  160nm_ficinv00aa1n08x5       g004(.clk(\a[3] ), .clkout(new_n100));
  160nm_ficinv00aa1n08x5       g005(.clk(\b[2] ), .clkout(new_n101));
  nanp02aa1n02x5               g006(.a(new_n101), .b(new_n100), .o1(new_n102));
  nanp02aa1n02x5               g007(.a(\b[2] ), .b(\a[3] ), .o1(new_n103));
  nanp02aa1n02x5               g008(.a(new_n102), .b(new_n103), .o1(new_n104));
  norp02aa1n02x5               g009(.a(\b[1] ), .b(\a[2] ), .o1(new_n105));
  nanp02aa1n02x5               g010(.a(\b[1] ), .b(\a[2] ), .o1(new_n106));
  nanp02aa1n02x5               g011(.a(\b[0] ), .b(\a[1] ), .o1(new_n107));
  aoi012aa1n02x5               g012(.a(new_n105), .b(new_n106), .c(new_n107), .o1(new_n108));
  160nm_ficinv00aa1n08x5       g013(.clk(\a[4] ), .clkout(new_n109));
  aboi22aa1n03x5               g014(.a(\b[3] ), .b(new_n109), .c(new_n100), .d(new_n101), .out0(new_n110));
  oaoi13aa1n02x5               g015(.a(new_n99), .b(new_n110), .c(new_n108), .d(new_n104), .o1(new_n111));
  norp02aa1n02x5               g016(.a(\b[7] ), .b(\a[8] ), .o1(new_n112));
  nanp02aa1n02x5               g017(.a(\b[7] ), .b(\a[8] ), .o1(new_n113));
  norp02aa1n02x5               g018(.a(\b[6] ), .b(\a[7] ), .o1(new_n114));
  nanp02aa1n02x5               g019(.a(\b[6] ), .b(\a[7] ), .o1(new_n115));
  nona23aa1n02x4               g020(.a(new_n115), .b(new_n113), .c(new_n112), .d(new_n114), .out0(new_n116));
  xnrc02aa1n02x5               g021(.a(\b[5] ), .b(\a[6] ), .out0(new_n117));
  xnrc02aa1n02x5               g022(.a(\b[4] ), .b(\a[5] ), .out0(new_n118));
  norp03aa1n02x5               g023(.a(new_n116), .b(new_n117), .c(new_n118), .o1(new_n119));
  oai022aa1n02x5               g024(.a(\a[5] ), .b(\b[4] ), .c(\b[5] ), .d(\a[6] ), .o1(new_n120));
  aob012aa1n02x5               g025(.a(new_n120), .b(\b[5] ), .c(\a[6] ), .out0(new_n121));
  160nm_ficinv00aa1n08x5       g026(.clk(\a[7] ), .clkout(new_n122));
  160nm_ficinv00aa1n08x5       g027(.clk(\b[6] ), .clkout(new_n123));
  aoai13aa1n02x5               g028(.a(new_n113), .b(new_n112), .c(new_n122), .d(new_n123), .o1(new_n124));
  oai012aa1n02x5               g029(.a(new_n124), .b(new_n116), .c(new_n121), .o1(new_n125));
  xorc02aa1n02x5               g030(.a(\a[9] ), .b(\b[8] ), .out0(new_n126));
  aoai13aa1n02x5               g031(.a(new_n126), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n127));
  xnbna2aa1n03x5               g032(.a(new_n97), .b(new_n127), .c(new_n98), .out0(\s[10] ));
  aobi12aa1n02x5               g033(.a(new_n97), .b(new_n127), .c(new_n98), .out0(new_n129));
  norp02aa1n02x5               g034(.a(\b[10] ), .b(\a[11] ), .o1(new_n130));
  nanp02aa1n02x5               g035(.a(\b[10] ), .b(\a[11] ), .o1(new_n131));
  norb02aa1n02x5               g036(.a(new_n131), .b(new_n130), .out0(new_n132));
  oaoi03aa1n02x5               g037(.a(\a[10] ), .b(\b[9] ), .c(new_n98), .o1(new_n133));
  oai012aa1n02x5               g038(.a(new_n132), .b(new_n129), .c(new_n133), .o1(new_n134));
  norp03aa1n02x5               g039(.a(new_n129), .b(new_n132), .c(new_n133), .o1(new_n135));
  norb02aa1n02x5               g040(.a(new_n134), .b(new_n135), .out0(\s[11] ));
  norp02aa1n02x5               g041(.a(\b[11] ), .b(\a[12] ), .o1(new_n137));
  nanp02aa1n02x5               g042(.a(\b[11] ), .b(\a[12] ), .o1(new_n138));
  norb02aa1n02x5               g043(.a(new_n138), .b(new_n137), .out0(new_n139));
  nona22aa1n02x4               g044(.a(new_n134), .b(new_n139), .c(new_n130), .out0(new_n140));
  160nm_ficinv00aa1n08x5       g045(.clk(new_n130), .clkout(new_n141));
  aobi12aa1n02x5               g046(.a(new_n139), .b(new_n134), .c(new_n141), .out0(new_n142));
  norb02aa1n02x5               g047(.a(new_n140), .b(new_n142), .out0(\s[12] ));
  nano23aa1n02x4               g048(.a(new_n130), .b(new_n137), .c(new_n138), .d(new_n131), .out0(new_n144));
  nanp03aa1n02x5               g049(.a(new_n144), .b(new_n97), .c(new_n126), .o1(new_n145));
  160nm_ficinv00aa1n08x5       g050(.clk(new_n145), .clkout(new_n146));
  aoai13aa1n02x5               g051(.a(new_n146), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n147));
  oai022aa1n02x5               g052(.a(\a[11] ), .b(\b[10] ), .c(\b[11] ), .d(\a[12] ), .o1(new_n148));
  aoi022aa1n02x5               g053(.a(new_n144), .b(new_n133), .c(new_n138), .d(new_n148), .o1(new_n149));
  xnrc02aa1n02x5               g054(.a(\b[12] ), .b(\a[13] ), .out0(new_n150));
  xobna2aa1n03x5               g055(.a(new_n150), .b(new_n147), .c(new_n149), .out0(\s[13] ));
  orn002aa1n02x5               g056(.a(\a[13] ), .b(\b[12] ), .o(new_n152));
  aoai13aa1n02x5               g057(.a(new_n152), .b(new_n150), .c(new_n147), .d(new_n149), .o1(new_n153));
  xorb03aa1n02x5               g058(.a(new_n153), .b(\b[13] ), .c(\a[14] ), .out0(\s[14] ));
  xnrc02aa1n02x5               g059(.a(\b[13] ), .b(\a[14] ), .out0(new_n155));
  norp02aa1n02x5               g060(.a(new_n155), .b(new_n150), .o1(new_n156));
  160nm_ficinv00aa1n08x5       g061(.clk(new_n156), .clkout(new_n157));
  oao003aa1n02x5               g062(.a(\a[14] ), .b(\b[13] ), .c(new_n152), .carry(new_n158));
  aoai13aa1n02x5               g063(.a(new_n158), .b(new_n157), .c(new_n147), .d(new_n149), .o1(new_n159));
  xorb03aa1n02x5               g064(.a(new_n159), .b(\b[14] ), .c(\a[15] ), .out0(\s[15] ));
  norp02aa1n02x5               g065(.a(\b[14] ), .b(\a[15] ), .o1(new_n161));
  nanp02aa1n02x5               g066(.a(\b[14] ), .b(\a[15] ), .o1(new_n162));
  norp02aa1n02x5               g067(.a(\b[15] ), .b(\a[16] ), .o1(new_n163));
  nanp02aa1n02x5               g068(.a(\b[15] ), .b(\a[16] ), .o1(new_n164));
  norb02aa1n02x5               g069(.a(new_n164), .b(new_n163), .out0(new_n165));
  aoi112aa1n02x5               g070(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n166));
  aoai13aa1n02x5               g071(.a(new_n165), .b(new_n161), .c(new_n159), .d(new_n162), .o1(new_n167));
  norb02aa1n02x5               g072(.a(new_n167), .b(new_n166), .out0(\s[16] ));
  nano23aa1n02x4               g073(.a(new_n161), .b(new_n163), .c(new_n164), .d(new_n162), .out0(new_n169));
  nona22aa1n02x4               g074(.a(new_n169), .b(new_n155), .c(new_n150), .out0(new_n170));
  norp02aa1n02x5               g075(.a(new_n170), .b(new_n145), .o1(new_n171));
  aoai13aa1n02x5               g076(.a(new_n171), .b(new_n125), .c(new_n111), .d(new_n119), .o1(new_n172));
  oai012aa1n02x5               g077(.a(new_n164), .b(new_n163), .c(new_n161), .o1(new_n173));
  oaib12aa1n02x5               g078(.a(new_n173), .b(new_n158), .c(new_n169), .out0(new_n174));
  oab012aa1n02x4               g079(.a(new_n174), .b(new_n149), .c(new_n170), .out0(new_n175));
  nanp02aa1n02x5               g080(.a(new_n172), .b(new_n175), .o1(new_n176));
  xorb03aa1n02x5               g081(.a(new_n176), .b(\b[16] ), .c(\a[17] ), .out0(\s[17] ));
  160nm_ficinv00aa1n08x5       g082(.clk(\a[18] ), .clkout(new_n178));
  160nm_ficinv00aa1n08x5       g083(.clk(\a[17] ), .clkout(new_n179));
  160nm_ficinv00aa1n08x5       g084(.clk(\b[16] ), .clkout(new_n180));
  oaoi03aa1n02x5               g085(.a(new_n179), .b(new_n180), .c(new_n176), .o1(new_n181));
  xorb03aa1n02x5               g086(.a(new_n181), .b(\b[17] ), .c(new_n178), .out0(\s[18] ));
  xroi22aa1d04x5               g087(.a(new_n179), .b(\b[16] ), .c(new_n178), .d(\b[17] ), .out0(new_n183));
  nanp02aa1n02x5               g088(.a(new_n180), .b(new_n179), .o1(new_n184));
  oaoi03aa1n02x5               g089(.a(\a[18] ), .b(\b[17] ), .c(new_n184), .o1(new_n185));
  norp02aa1n02x5               g090(.a(\b[18] ), .b(\a[19] ), .o1(new_n186));
  nanp02aa1n02x5               g091(.a(\b[18] ), .b(\a[19] ), .o1(new_n187));
  norb02aa1n02x5               g092(.a(new_n187), .b(new_n186), .out0(new_n188));
  aoai13aa1n02x5               g093(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n189));
  aoi112aa1n02x5               g094(.a(new_n188), .b(new_n185), .c(new_n176), .d(new_n183), .o1(new_n190));
  norb02aa1n02x5               g095(.a(new_n189), .b(new_n190), .out0(\s[19] ));
  xnrc02aa1n02x5               g096(.a(\b[0] ), .b(\a[1] ), .out0(\s[1] ));
  norp02aa1n02x5               g097(.a(\b[19] ), .b(\a[20] ), .o1(new_n193));
  nanp02aa1n02x5               g098(.a(\b[19] ), .b(\a[20] ), .o1(new_n194));
  norb02aa1n02x5               g099(.a(new_n194), .b(new_n193), .out0(new_n195));
  nona22aa1n02x4               g100(.a(new_n189), .b(new_n195), .c(new_n186), .out0(new_n196));
  160nm_ficinv00aa1n08x5       g101(.clk(new_n195), .clkout(new_n197));
  oaoi13aa1n02x5               g102(.a(new_n197), .b(new_n189), .c(\a[19] ), .d(\b[18] ), .o1(new_n198));
  norb02aa1n02x5               g103(.a(new_n196), .b(new_n198), .out0(\s[20] ));
  nano23aa1n02x4               g104(.a(new_n186), .b(new_n193), .c(new_n194), .d(new_n187), .out0(new_n200));
  nanp02aa1n02x5               g105(.a(new_n183), .b(new_n200), .o1(new_n201));
  oai022aa1n02x5               g106(.a(\a[17] ), .b(\b[16] ), .c(\b[17] ), .d(\a[18] ), .o1(new_n202));
  oaib12aa1n02x5               g107(.a(new_n202), .b(new_n178), .c(\b[17] ), .out0(new_n203));
  nona23aa1n02x4               g108(.a(new_n194), .b(new_n187), .c(new_n186), .d(new_n193), .out0(new_n204));
  aoi012aa1n02x5               g109(.a(new_n193), .b(new_n186), .c(new_n194), .o1(new_n205));
  oai012aa1n02x5               g110(.a(new_n205), .b(new_n204), .c(new_n203), .o1(new_n206));
  160nm_ficinv00aa1n08x5       g111(.clk(new_n206), .clkout(new_n207));
  aoai13aa1n02x5               g112(.a(new_n207), .b(new_n201), .c(new_n172), .d(new_n175), .o1(new_n208));
  xorb03aa1n02x5               g113(.a(new_n208), .b(\b[20] ), .c(\a[21] ), .out0(\s[21] ));
  norp02aa1n02x5               g114(.a(\b[20] ), .b(\a[21] ), .o1(new_n210));
  xorc02aa1n02x5               g115(.a(\a[21] ), .b(\b[20] ), .out0(new_n211));
  xorc02aa1n02x5               g116(.a(\a[22] ), .b(\b[21] ), .out0(new_n212));
  aoi112aa1n02x5               g117(.a(new_n210), .b(new_n212), .c(new_n208), .d(new_n211), .o1(new_n213));
  aoai13aa1n02x5               g118(.a(new_n212), .b(new_n210), .c(new_n208), .d(new_n211), .o1(new_n214));
  norb02aa1n02x5               g119(.a(new_n214), .b(new_n213), .out0(\s[22] ));
  160nm_ficinv00aa1n08x5       g120(.clk(\a[21] ), .clkout(new_n216));
  160nm_ficinv00aa1n08x5       g121(.clk(\a[22] ), .clkout(new_n217));
  xroi22aa1d04x5               g122(.a(new_n216), .b(\b[20] ), .c(new_n217), .d(\b[21] ), .out0(new_n218));
  nanp03aa1n02x5               g123(.a(new_n218), .b(new_n183), .c(new_n200), .o1(new_n219));
  160nm_ficinv00aa1n08x5       g124(.clk(\b[21] ), .clkout(new_n220));
  oaoi03aa1n02x5               g125(.a(new_n217), .b(new_n220), .c(new_n210), .o1(new_n221));
  160nm_ficinv00aa1n08x5       g126(.clk(new_n221), .clkout(new_n222));
  aoi012aa1n02x5               g127(.a(new_n222), .b(new_n206), .c(new_n218), .o1(new_n223));
  aoai13aa1n02x5               g128(.a(new_n223), .b(new_n219), .c(new_n172), .d(new_n175), .o1(new_n224));
  xorb03aa1n02x5               g129(.a(new_n224), .b(\b[22] ), .c(\a[23] ), .out0(\s[23] ));
  norp02aa1n02x5               g130(.a(\b[22] ), .b(\a[23] ), .o1(new_n226));
  xorc02aa1n02x5               g131(.a(\a[23] ), .b(\b[22] ), .out0(new_n227));
  xorc02aa1n02x5               g132(.a(\a[24] ), .b(\b[23] ), .out0(new_n228));
  aoi112aa1n02x5               g133(.a(new_n226), .b(new_n228), .c(new_n224), .d(new_n227), .o1(new_n229));
  aoai13aa1n02x5               g134(.a(new_n228), .b(new_n226), .c(new_n224), .d(new_n227), .o1(new_n230));
  norb02aa1n02x5               g135(.a(new_n230), .b(new_n229), .out0(\s[24] ));
  and002aa1n02x5               g136(.a(new_n228), .b(new_n227), .o(new_n232));
  160nm_ficinv00aa1n08x5       g137(.clk(new_n232), .clkout(new_n233));
  nano32aa1n02x4               g138(.a(new_n233), .b(new_n218), .c(new_n183), .d(new_n200), .out0(new_n234));
  160nm_ficinv00aa1n08x5       g139(.clk(new_n205), .clkout(new_n235));
  aoai13aa1n02x5               g140(.a(new_n218), .b(new_n235), .c(new_n200), .d(new_n185), .o1(new_n236));
  orn002aa1n02x5               g141(.a(\a[23] ), .b(\b[22] ), .o(new_n237));
  oao003aa1n02x5               g142(.a(\a[24] ), .b(\b[23] ), .c(new_n237), .carry(new_n238));
  aoai13aa1n02x5               g143(.a(new_n238), .b(new_n233), .c(new_n236), .d(new_n221), .o1(new_n239));
  xorc02aa1n02x5               g144(.a(\a[25] ), .b(\b[24] ), .out0(new_n240));
  aoai13aa1n02x5               g145(.a(new_n240), .b(new_n239), .c(new_n176), .d(new_n234), .o1(new_n241));
  aoi112aa1n02x5               g146(.a(new_n240), .b(new_n239), .c(new_n176), .d(new_n234), .o1(new_n242));
  norb02aa1n02x5               g147(.a(new_n241), .b(new_n242), .out0(\s[25] ));
  norp02aa1n02x5               g148(.a(\b[24] ), .b(\a[25] ), .o1(new_n244));
  xorc02aa1n02x5               g149(.a(\a[26] ), .b(\b[25] ), .out0(new_n245));
  nona22aa1n02x4               g150(.a(new_n241), .b(new_n245), .c(new_n244), .out0(new_n246));
  160nm_ficinv00aa1n08x5       g151(.clk(new_n244), .clkout(new_n247));
  aobi12aa1n02x5               g152(.a(new_n245), .b(new_n241), .c(new_n247), .out0(new_n248));
  norb02aa1n02x5               g153(.a(new_n246), .b(new_n248), .out0(\s[26] ));
  nanp02aa1n02x5               g154(.a(new_n111), .b(new_n119), .o1(new_n250));
  160nm_ficinv00aa1n08x5       g155(.clk(new_n125), .clkout(new_n251));
  nanp02aa1n02x5               g156(.a(new_n250), .b(new_n251), .o1(new_n252));
  oabi12aa1n02x5               g157(.a(new_n174), .b(new_n149), .c(new_n170), .out0(new_n253));
  and002aa1n02x5               g158(.a(new_n245), .b(new_n240), .o(new_n254));
  nano22aa1n02x4               g159(.a(new_n219), .b(new_n232), .c(new_n254), .out0(new_n255));
  aoai13aa1n02x5               g160(.a(new_n255), .b(new_n253), .c(new_n252), .d(new_n171), .o1(new_n256));
  oao003aa1n02x5               g161(.a(\a[26] ), .b(\b[25] ), .c(new_n247), .carry(new_n257));
  aobi12aa1n02x5               g162(.a(new_n257), .b(new_n239), .c(new_n254), .out0(new_n258));
  xorc02aa1n02x5               g163(.a(\a[27] ), .b(\b[26] ), .out0(new_n259));
  xnbna2aa1n03x5               g164(.a(new_n259), .b(new_n258), .c(new_n256), .out0(\s[27] ));
  norp02aa1n02x5               g165(.a(\b[26] ), .b(\a[27] ), .o1(new_n261));
  160nm_ficinv00aa1n08x5       g166(.clk(new_n261), .clkout(new_n262));
  aobi12aa1n02x5               g167(.a(new_n259), .b(new_n258), .c(new_n256), .out0(new_n263));
  xnrc02aa1n02x5               g168(.a(\b[27] ), .b(\a[28] ), .out0(new_n264));
  nano22aa1n02x4               g169(.a(new_n263), .b(new_n262), .c(new_n264), .out0(new_n265));
  aobi12aa1n02x5               g170(.a(new_n255), .b(new_n172), .c(new_n175), .out0(new_n266));
  aoai13aa1n02x5               g171(.a(new_n232), .b(new_n222), .c(new_n206), .d(new_n218), .o1(new_n267));
  160nm_ficinv00aa1n08x5       g172(.clk(new_n254), .clkout(new_n268));
  aoai13aa1n02x5               g173(.a(new_n257), .b(new_n268), .c(new_n267), .d(new_n238), .o1(new_n269));
  oai012aa1n02x5               g174(.a(new_n259), .b(new_n269), .c(new_n266), .o1(new_n270));
  aoi012aa1n02x5               g175(.a(new_n264), .b(new_n270), .c(new_n262), .o1(new_n271));
  norp02aa1n02x5               g176(.a(new_n271), .b(new_n265), .o1(\s[28] ));
  norb02aa1n02x5               g177(.a(new_n259), .b(new_n264), .out0(new_n273));
  aobi12aa1n02x5               g178(.a(new_n273), .b(new_n258), .c(new_n256), .out0(new_n274));
  oao003aa1n02x5               g179(.a(\a[28] ), .b(\b[27] ), .c(new_n262), .carry(new_n275));
  xnrc02aa1n02x5               g180(.a(\b[28] ), .b(\a[29] ), .out0(new_n276));
  nano22aa1n02x4               g181(.a(new_n274), .b(new_n275), .c(new_n276), .out0(new_n277));
  oai012aa1n02x5               g182(.a(new_n273), .b(new_n269), .c(new_n266), .o1(new_n278));
  aoi012aa1n02x5               g183(.a(new_n276), .b(new_n278), .c(new_n275), .o1(new_n279));
  norp02aa1n02x5               g184(.a(new_n279), .b(new_n277), .o1(\s[29] ));
  xorb03aa1n02x5               g185(.a(new_n107), .b(\b[1] ), .c(\a[2] ), .out0(\s[2] ));
  norb03aa1n02x5               g186(.a(new_n259), .b(new_n276), .c(new_n264), .out0(new_n282));
  aobi12aa1n02x5               g187(.a(new_n282), .b(new_n258), .c(new_n256), .out0(new_n283));
  oao003aa1n02x5               g188(.a(\a[29] ), .b(\b[28] ), .c(new_n275), .carry(new_n284));
  xnrc02aa1n02x5               g189(.a(\b[29] ), .b(\a[30] ), .out0(new_n285));
  nano22aa1n02x4               g190(.a(new_n283), .b(new_n284), .c(new_n285), .out0(new_n286));
  oai012aa1n02x5               g191(.a(new_n282), .b(new_n269), .c(new_n266), .o1(new_n287));
  aoi012aa1n02x5               g192(.a(new_n285), .b(new_n287), .c(new_n284), .o1(new_n288));
  norp02aa1n02x5               g193(.a(new_n288), .b(new_n286), .o1(\s[30] ));
  norb02aa1n02x5               g194(.a(new_n282), .b(new_n285), .out0(new_n290));
  aobi12aa1n02x5               g195(.a(new_n290), .b(new_n258), .c(new_n256), .out0(new_n291));
  oao003aa1n02x5               g196(.a(\a[30] ), .b(\b[29] ), .c(new_n284), .carry(new_n292));
  xnrc02aa1n02x5               g197(.a(\b[30] ), .b(\a[31] ), .out0(new_n293));
  nano22aa1n02x4               g198(.a(new_n291), .b(new_n292), .c(new_n293), .out0(new_n294));
  oai012aa1n02x5               g199(.a(new_n290), .b(new_n269), .c(new_n266), .o1(new_n295));
  aoi012aa1n02x5               g200(.a(new_n293), .b(new_n295), .c(new_n292), .o1(new_n296));
  norp02aa1n02x5               g201(.a(new_n296), .b(new_n294), .o1(\s[31] ));
  xnbna2aa1n03x5               g202(.a(new_n108), .b(new_n102), .c(new_n103), .out0(\s[3] ));
  oaoi03aa1n02x5               g203(.a(\a[3] ), .b(\b[2] ), .c(new_n108), .o1(new_n299));
  xorb03aa1n02x5               g204(.a(new_n299), .b(\b[3] ), .c(\a[4] ), .out0(\s[4] ));
  xorb03aa1n02x5               g205(.a(new_n111), .b(\b[4] ), .c(\a[5] ), .out0(\s[5] ));
  160nm_ficinv00aa1n08x5       g206(.clk(new_n111), .clkout(new_n302));
  oaoi03aa1n02x5               g207(.a(\a[5] ), .b(\b[4] ), .c(new_n302), .o1(new_n303));
  xorb03aa1n02x5               g208(.a(new_n303), .b(\b[5] ), .c(\a[6] ), .out0(\s[6] ));
  oai013aa1n02x4               g209(.a(new_n121), .b(new_n302), .c(new_n117), .d(new_n118), .o1(new_n305));
  xorb03aa1n02x5               g210(.a(new_n305), .b(\b[6] ), .c(\a[7] ), .out0(\s[7] ));
  oaoi03aa1n02x5               g211(.a(new_n122), .b(new_n123), .c(new_n305), .o1(new_n307));
  xnrb03aa1n02x5               g212(.a(new_n307), .b(\b[7] ), .c(\a[8] ), .out0(\s[8] ));
  xnbna2aa1n03x5               g213(.a(new_n126), .b(new_n250), .c(new_n251), .out0(\s[9] ));
endmodule


